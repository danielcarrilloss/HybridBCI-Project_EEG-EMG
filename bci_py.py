import time
import threading
import serial
import numpy as np
from pylsl import StreamInlet, resolve_byprop
from scipy.signal import welch, butter, lfilter, iirnotch
import msvcrt

# =========================================================
# SERIAL / LSL CONFIG
# =========================================================
SERIAL_PORT = "COM6"
BAUDRATE = 115200

FS = 256
MU_BAND = [8, 13]
WINDOW_SEC = 3
BUFFER_SAMPLES = FS * WINDOW_SEC

# 8 EEG channels used from the 9-channel Bitbrain stream
EEG_CHANNELS_TO_USE = 8

# Stream channel order:
# 0 FC3, 1 FCz, 2 FC4, 3 C3, 4 Cz, 5 C4, 6 CP3, 7 CP4
CHANNEL_NAMES = ["FC3", "FCz", "FC4", "C3", "Cz", "C4", "CP3", "CP4"]

# Classification uses ONLY the motor channels
FEATURE_CH_IDX = [3, 4, 5]   # C3, Cz, C4
FEATURE_NAMES = ["C3", "Cz", "C4"]

# Motor channels for quick scalar debug
MOTOR_CH_IDX = FEATURE_CH_IDX

# =========================================================
# EEG / CLASSIFICATION CONFIG
# =========================================================
REST_CAL_SEC = 15

# Longer and more stable class calibration
CLASS_TRIALS = 8
CLASS_REST_SEC = 2
CLASS_IMAGINE_SEC = 6

PROCESS_INTERVAL_SEC = 0.20
ROBOT_COOLDOWN_SEC = 15

# After pressing E:
# countdown first, then this much silent signal collection before decisions begin
DECISION_WARMUP_SEC = 5.0

# Easier debugging than 5
REQUIRED_CONSECUTIVE = 3

# Rejection logic
CLASS_MARGIN = 0.05
MIN_PROTO_REST_DISTANCE = 0.12
MIN_COMMAND_REST_DISTANCE = 0.03
MIN_ACCEPT_THR = 0.05
MAX_ACCEPT_THR = 5.0

CLASS1_NAME = "HAND"
CLASS2_NAME = "FEET"

CLASS1_INSTRUCTION = "Imagine repeatedly squeezing a ball with your dominant hand."
CLASS2_INSTRUCTION = "Imagine both feet dorsiflexing rhythmically, as if lifting your toes upward."

# Default: focus first on EEG-only route validation
AUTO_OUTPUT_MODE = "direct"   # "direct" or "hybrid"

# =========================================================
# ARDUINO COMMANDS
# =========================================================
# Hybrid routes
CMD_HYBRID_ROUTE1 = b'A'
CMD_HYBRID_ROUTE2 = b'H'

# Direct routes
CMD_DIRECT_ROUTE1 = b'D'
CMD_DIRECT_ROUTE2 = b'G'

# EMG recalibration
CMD_RECAL_EMG = b'C'

# =========================================================
# FILTERS
# =========================================================
def bandpass_filter(data, lowcut, highcut, fs=FS, order=4):
    nyq = 0.5 * fs
    b, a = butter(order, [lowcut / nyq, highcut / nyq], btype='bandpass')
    return lfilter(b, a, data, axis=0)


def notch_filter(data, fs=FS, freq=50):
    nyq = 0.5 * fs
    w0 = freq / nyq
    b, a = iirnotch(w0, Q=30)
    return lfilter(b, a, data, axis=0)

# =========================================================
# EEG BUFFER / ACQUISITION HELPERS
# =========================================================
def reset_eeg_buffer():
    return np.zeros((BUFFER_SAMPLES, EEG_CHANNELS_TO_USE), dtype=float)


def update_buffer(data_buffer, sample):
    """Insert one newest sample into rolling buffer."""
    eeg_sample = np.array(sample[:EEG_CHANNELS_TO_USE], dtype=float)
    data_buffer = np.roll(data_buffer, -1, axis=0)
    data_buffer[-1, :] = eeg_sample
    return data_buffer


def update_buffer_chunk(data_buffer, samples):
    """Insert a chunk of samples into rolling buffer."""
    if samples is None or len(samples) == 0:
        return data_buffer

    arr = np.asarray(samples, dtype=float)
    arr = arr[:, :EEG_CHANNELS_TO_USE]

    n = arr.shape[0]
    if n >= BUFFER_SAMPLES:
        data_buffer = arr[-BUFFER_SAMPLES:, :]
    else:
        data_buffer = np.roll(data_buffer, -n, axis=0)
        data_buffer[-n:, :] = arr

    return data_buffer


def pull_available_eeg(inlet, data_buffer, max_samples=64):
    """
    Pull all currently available EEG samples without blocking.
    Better than a single pull_sample because it avoids falling behind.
    """
    samples, _ = inlet.pull_chunk(timeout=0.0, max_samples=max_samples)
    if samples:
        data_buffer = update_buffer_chunk(data_buffer, samples)
    return data_buffer


def consume_stream_seconds(inlet, seconds, data_buffer=None, update=False, fs=FS):
    """
    Consume EEG stream for a given number of seconds.
    If update=True and data_buffer is provided, samples are inserted into buffer.
    """
    n_samples = int(seconds * fs)
    if data_buffer is None:
        data_buffer = reset_eeg_buffer()

    for _ in range(n_samples):
        sample, _ = inlet.pull_sample(timeout=1.0)
        if sample is None:
            continue
        if update:
            data_buffer = update_buffer(data_buffer, sample)

    return data_buffer


def countdown_stream(inlet, seconds=3, prefix="[EEG] Starting in"):
    """
    Countdown while still consuming the stream, so LSL does not backlog.
    Samples are discarded here on purpose.
    """
    for k in range(seconds, 0, -1):
        print(f"{prefix} {k}...")
        consume_stream_seconds(inlet, 1.0, update=False)

# =========================================================
# EEG FEATURE FUNCTIONS
# =========================================================
def get_mu_vector(data_chunk, channel_idx=FEATURE_CH_IDX):
    """
    Returns mean Mu-band power per selected channel.
    Default = motor channels only: C3, Cz, C4
    """
    selected = data_chunk[:, channel_idx]

    # CAR over selected channels only
    car = selected - np.mean(selected, axis=1, keepdims=True)

    clean = notch_filter(car)
    clean = bandpass_filter(clean, MU_BAND[0], MU_BAND[1])

    freqs, psd = welch(clean, FS, nperseg=FS, axis=0)
    idx = (freqs >= MU_BAND[0]) & (freqs <= MU_BAND[1])

    return np.mean(psd[idx, :], axis=0)


def get_mu_power_motor(data_chunk):
    """Average Mu power over classifier channels."""
    mu_vec = get_mu_vector(data_chunk, FEATURE_CH_IDX)
    return float(np.mean(mu_vec))


def normalize_feature(vec, rest_vec, eps=1e-6):
    """
    Robust normalization using log-ratio:
    - avoids exploding ratios when a rest channel is very small
    - normalized rest is near 0 instead of 1
    """
    rest_floor = max(np.median(rest_vec) * 0.2, eps)
    rest_safe = np.maximum(rest_vec, rest_floor)
    return np.log((vec + eps) / (rest_safe + eps))


def format_vec(vec, names=None):
    if names is None:
        if len(vec) == len(FEATURE_NAMES):
            names = FEATURE_NAMES
        else:
            names = CHANNEL_NAMES[:len(vec)]
    return " | ".join(f"{ch}:{val:.3f}" for ch, val in zip(names, vec))

# =========================================================
# SERIAL HELPERS
# =========================================================
def arduino_reader(ser, robot_state):
    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode(errors="ignore").strip()
                if line:
                    print(f"[ARDUINO] {line}")

                    if line == "ROBOT_BUSY=1":
                        robot_state["busy"] = True
                    elif line == "ROBOT_BUSY=0":
                        robot_state["busy"] = False
            else:
                time.sleep(0.02)
        except Exception:
            break


def send_command(ser, cmd):
    ser.write(cmd)
    ser.flush()
    print(f"[PYTHON] Sent command: {cmd.decode()}")


def route_command_for_label(label, output_mode):
    if output_mode == "direct":
        return CMD_DIRECT_ROUTE1 if label == CLASS1_NAME else CMD_DIRECT_ROUTE2
    else:
        return CMD_HYBRID_ROUTE1 if label == CLASS1_NAME else CMD_HYBRID_ROUTE2


def send_route_for_label(ser, label, output_mode):
    cmd = route_command_for_label(label, output_mode)
    send_command(ser, cmd)

# =========================================================
# CALIBRATION FUNCTIONS
# =========================================================
def calibrate_eeg_rest(inlet, data_buffer, seconds=REST_CAL_SEC, fs=FS):
    """
    Rest calibration:
    - resets buffer for a clean segment
    - gives the user a countdown
    - builds rest_vec over classifier channels only (C3, Cz, C4)
    - builds baseline_power over the same channels
    """
    print(f"\n[EEG] Rest calibration: relax for {seconds} seconds...")
    print("[EEG] No online results will be printed until calibration is fully finished.")
    countdown_stream(inlet, 3, prefix="[EEG] Rest calibration starts in")

    data_buffer = reset_eeg_buffer()

    rest_feats = []
    motor_powers = []

    n_samples = int(seconds * fs)
    for i in range(n_samples):
        sample, _ = inlet.pull_sample(timeout=1.0)
        if sample is None:
            continue

        data_buffer = update_buffer(data_buffer, sample)

        # start extracting once the 3-second buffer is full
        if i >= BUFFER_SAMPLES and i % 32 == 0:
            rest_feats.append(get_mu_vector(data_buffer))
            motor_powers.append(get_mu_power_motor(data_buffer))

    if len(rest_feats) == 0 or len(motor_powers) == 0:
        print("[EEG] Rest calibration failed.")
        return None, None, data_buffer

    rest_vec = np.mean(np.array(rest_feats), axis=0)
    baseline_power = float(np.mean(motor_powers))

    print(f"[EEG] New rest baseline (motor): {baseline_power:.6f}")
    print(f"[EEG] Rest raw vector: {format_vec(rest_vec, FEATURE_NAMES)}")

    return baseline_power, rest_vec, data_buffer


def calibrate_class_prototype(
    inlet,
    data_buffer,
    rest_vec,
    label,
    instruction,
    n_trials=CLASS_TRIALS,
    rest_sec=CLASS_REST_SEC,
    imagine_sec=CLASS_IMAGINE_SEC,
    fs=FS
):
    """
    Trial-based class calibration:
    each trial = short rest + countdown + clean imagery block
    Returns a class_info dict.
    """
    if rest_vec is None:
        print("[EEG] First do rest calibration with R.")
        return None, data_buffer

    print(f"\n[EEG] Calibrating class: {label}")
    print(f"[EEG] Instruction: {instruction}")
    print(f"[EEG] Trials: {n_trials} | rest {rest_sec}s + imagine {imagine_sec}s")
    print("[EEG] No online results will be printed until calibration is fully finished.")

    feats = []

    for tr in range(n_trials):
        print(f"\n[EEG] Trial {tr+1}/{n_trials}")
        print(f"[EEG] REST for {rest_sec} s...")
        consume_stream_seconds(inlet, rest_sec, update=False)

        countdown_stream(inlet, 3, prefix="[EEG] Imagine starts in")

        # fresh imagery buffer: only post-countdown imagery enters the window
        data_buffer = reset_eeg_buffer()

        print(f"[EEG] GO -> {instruction}")
        n_imagine = int(imagine_sec * fs)
        for i in range(n_imagine):
            sample, _ = inlet.pull_sample(timeout=1.0)
            if sample is None:
                continue

            data_buffer = update_buffer(data_buffer, sample)

            # only extract once imagery buffer is truly full
            if i >= BUFFER_SAMPLES and i % 32 == 0:
                feat = get_mu_vector(data_buffer)
                feat_norm = normalize_feature(feat, rest_vec)
                feats.append(feat_norm)

    if len(feats) == 0:
        print(f"[EEG] Calibration failed for {label}.")
        return None, data_buffer

    feats = np.array(feats)
    proto = np.mean(feats, axis=0)

    # Training-window distance to prototype
    dists = np.linalg.norm(feats - proto, axis=1)
    dist_mean = float(np.mean(dists))
    dist_std = float(np.std(dists))
    accept_thr = dist_mean + 2.0 * dist_std
    accept_thr = max(MIN_ACCEPT_THR, min(MAX_ACCEPT_THR, accept_thr))

    # Distance to normalized rest reference (= zeros in log-ratio space)
    rest_ref = np.zeros_like(proto)
    proto_rest_dist = float(np.linalg.norm(proto - rest_ref))
    usable = proto_rest_dist >= MIN_PROTO_REST_DISTANCE

    print(f"\n[EEG] {label} prototype stored.")
    print(f"[EEG] {label} norm prototype: {format_vec(proto, FEATURE_NAMES)}")
    print(f"[EEG] {label} distance-to-rest: {proto_rest_dist:.3f}")
    print(
        f"[EEG] {label} train dist mean={dist_mean:.3f}, "
        f"std={dist_std:.3f}, accept_thr={accept_thr:.3f}"
    )

    if not usable:
        print(f"[EEG][WARNING] {label} prototype is too close to rest.")
        print(f"[EEG][WARNING] Recalibrate {label}; this class should NOT be trusted yet.")

    class_info = {
        "label": label,
        "proto": proto,
        "accept_thr": accept_thr,
        "proto_rest_dist": proto_rest_dist,
        "dist_mean": dist_mean,
        "dist_std": dist_std,
        "usable": usable,
    }

    return class_info, data_buffer

# =========================================================
# CLASSIFICATION
# =========================================================
def classify_current_window(data_buffer, rest_vec, class1_info, class2_info):
    feat = get_mu_vector(data_buffer)
    feat_norm = normalize_feature(feat, rest_vec)

    d1 = float(np.linalg.norm(feat_norm - class1_info["proto"]))
    d2 = float(np.linalg.norm(feat_norm - class2_info["proto"]))

    # In log-ratio space, rest is around zero
    rest_ref = np.zeros_like(feat_norm)
    feat_rest_dist = float(np.linalg.norm(feat_norm - rest_ref))

    if d1 < d2:
        best_label = class1_info["label"]
        best_dist = d1
        other_dist = d2
        best_thr = class1_info["accept_thr"]
    else:
        best_label = class2_info["label"]
        best_dist = d2
        other_dist = d1
        best_thr = class2_info["accept_thr"]

    margin = other_dist - best_dist

    valid = (
        feat_rest_dist > MIN_COMMAND_REST_DISTANCE and
        best_dist < best_thr and
        margin > CLASS_MARGIN
    )

    return {
        "feat_norm": feat_norm,
        "d1": d1,
        "d2": d2,
        "feat_rest_dist": feat_rest_dist,
        "best_label": best_label,
        "best_dist": best_dist,
        "other_dist": other_dist,
        "best_thr": best_thr,
        "margin": margin,
        "valid": valid
    }

# =========================================================
# UI
# =========================================================
def print_help(output_mode):
    print("\nSistema activo.")
    print("Manual route commands:")
    print("  D -> Direct Route 1 (EEG-only validation)")
    print("  F -> Direct Route 2 (EEG-only validation)")
    print("  S -> Hybrid Route 1 (EMG-assisted)")
    print("  X -> Hybrid Route 2 (EMG-assisted)")
    print("")
    print("Calibration / control:")
    print("  R -> Recalibrate EEG rest baseline")
    print(f"  1 -> Calibrate class 1 ({CLASS1_NAME})")
    print(f"  2 -> Calibrate class 2 ({CLASS2_NAME})")
    print("  C -> Recalibrate EMG on Arduino")
    print("  E -> Arm/disarm automatic EEG route selection")
    print(f"  T -> Toggle EEG automatic output mode (current: {output_mode})")
    print("  Q -> Quit\n")

# =========================================================
# MAIN
# =========================================================
def main():
    eeg_auto_enabled = False
    auto_output_mode = AUTO_OUTPUT_MODE

    baseline_power = None
    rest_vec = None
    class1_info = None
    class2_info = None

    last_process_time = time.time()
    robot_busy_until = 0.0
    decision_ready_at = 0.0

    consecutive_label = None
    consecutive_count = 0

    robot_state = {"busy": False}

    # --------------------------
    # Connect Arduino
    # --------------------------
    print("Connecting to Arduino...")
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=0.1)
    time.sleep(2.0)
    print("Connected to Arduino.")

    threading.Thread(target=arduino_reader, args=(ser, robot_state), daemon=True).start()

    # --------------------------
    # Connect EEG
    # --------------------------
    print("Buscando stream EEG por tipo...")
    streams = resolve_byprop("type", "EEG", timeout=10)
    if not streams:
        print("No se encontró ningún stream EEG")
        ser.close()
        return

    eeg_stream = streams[0]
    inlet = StreamInlet(eeg_stream)

    print(f"Conectado al stream: {eeg_stream.name()}")
    print(f"Canales detectados: {eeg_stream.channel_count()}")
    print(f"Frecuencia detectada: {eeg_stream.nominal_srate()} Hz")

    if eeg_stream.channel_count() < EEG_CHANNELS_TO_USE:
        print(f"Error: el stream tiene menos de {EEG_CHANNELS_TO_USE} canales")
        ser.close()
        return

    data_buffer = reset_eeg_buffer()

    # --------------------------
    # Initial rest calibration
    # --------------------------
    baseline_power, rest_vec, data_buffer = calibrate_eeg_rest(
        inlet, data_buffer, seconds=REST_CAL_SEC
    )

    if baseline_power is None or rest_vec is None:
        print("[EEG] Initial rest calibration failed. Exiting.")
        ser.close()
        return

    print_help(auto_output_mode)

    try:
        while True:
            # ======================================================
            # 1) KEYBOARD HANDLING
            # ======================================================
            if msvcrt.kbhit():
                key = msvcrt.getch().lower()

                # ---- Manual direct Route 1
                if key == b'd':
                    now = time.time()
                    if robot_state["busy"] or now <= robot_busy_until:
                        print("[PYTHON] Robot busy / cooldown.")
                    else:
                        send_command(ser, CMD_DIRECT_ROUTE1)
                        robot_busy_until = now + 2.0

                # ---- Manual direct Route 2
                elif key == b'f':
                    now = time.time()
                    if robot_state["busy"] or now <= robot_busy_until:
                        print("[PYTHON] Robot busy / cooldown.")
                    else:
                        send_command(ser, CMD_DIRECT_ROUTE2)
                        robot_busy_until = now + 2.0

                # ---- Manual hybrid Route 1
                elif key == b's':
                    now = time.time()
                    if robot_state["busy"] or now <= robot_busy_until:
                        print("[PYTHON] Robot busy / cooldown.")
                    else:
                        send_command(ser, CMD_HYBRID_ROUTE1)
                        robot_busy_until = now + 2.0

                # ---- Manual hybrid Route 2
                elif key == b'x':
                    now = time.time()
                    if robot_state["busy"] or now <= robot_busy_until:
                        print("[PYTHON] Robot busy / cooldown.")
                    else:
                        send_command(ser, CMD_HYBRID_ROUTE2)
                        robot_busy_until = now + 2.0

                # ---- EEG rest recalibration
                elif key == b'r':
                    eeg_auto_enabled = False
                    consecutive_label = None
                    consecutive_count = 0

                    baseline_power, rest_vec, data_buffer = calibrate_eeg_rest(
                        inlet,
                        data_buffer,
                        seconds=REST_CAL_SEC
                    )

                    if baseline_power is not None and rest_vec is not None:
                        # Important: class prototypes were normalized to the old rest baseline
                        class1_info = None
                        class2_info = None
                        print("[PYTHON] EEG auto mode temporarily disabled after recalibration.")
                        print("[PYTHON] Rest baseline changed, so class 1 and class 2 must be recalibrated.")

                # ---- Class 1 calibration
                elif key == b'1':
                    eeg_auto_enabled = False
                    consecutive_label = None
                    consecutive_count = 0
                    class1_info, data_buffer = calibrate_class_prototype(
                        inlet,
                        data_buffer,
                        rest_vec,
                        CLASS1_NAME,
                        CLASS1_INSTRUCTION
                    )
                    print("[PYTHON] EEG auto mode temporarily disabled after class calibration.")

                # ---- Class 2 calibration
                elif key == b'2':
                    eeg_auto_enabled = False
                    consecutive_label = None
                    consecutive_count = 0
                    class2_info, data_buffer = calibrate_class_prototype(
                        inlet,
                        data_buffer,
                        rest_vec,
                        CLASS2_NAME,
                        CLASS2_INSTRUCTION
                    )
                    print("[PYTHON] EEG auto mode temporarily disabled after class calibration.")

                # ---- EMG recalibration
                elif key == b'c':
                    send_command(ser, CMD_RECAL_EMG)

                # ---- Toggle auto EEG route selection
                elif key == b'e':
                    if not eeg_auto_enabled:
                        if rest_vec is None or class1_info is None or class2_info is None:
                            print("[PYTHON] Cannot enable EEG auto mode yet.")
                            print("[PYTHON] Do R, then 1, then 2 first.")
                        elif not class1_info["usable"] or not class2_info["usable"]:
                            print("[PYTHON] Cannot enable EEG auto mode yet.")
                            print("[PYTHON] At least one class prototype is too close to rest.")
                            print("[PYTHON] Recalibrate the bad class.")
                        elif robot_state["busy"]:
                            print("[PYTHON] Robot is currently busy.")
                        else:
                            print(f"[PYTHON] EEG automatic route selection = ON ({auto_output_mode})")
                            print(f"[PYTHON] Think {CLASS1_NAME} = {CLASS1_INSTRUCTION}")
                            print(f"[PYTHON] Think {CLASS2_NAME} = {CLASS2_INSTRUCTION}")
                            countdown_stream(inlet, 3, prefix="[PYTHON] Decision starts in")

                            # clean decision segment
                            data_buffer = reset_eeg_buffer()

                            eeg_auto_enabled = True
                            consecutive_label = None
                            consecutive_count = 0
                            decision_ready_at = time.time() + DECISION_WARMUP_SEC

                            print("[PYTHON] GO. Keep imagining continuously.")
                            print(f"[PYTHON] First decision window will open after {DECISION_WARMUP_SEC:.1f} s.")
                    else:
                        eeg_auto_enabled = False
                        consecutive_label = None
                        consecutive_count = 0
                        print("[PYTHON] EEG automatic route selection = OFF")

                # ---- Toggle direct/hybrid output mode for automatic EEG
                elif key == b't':
                    auto_output_mode = "hybrid" if auto_output_mode == "direct" else "direct"
                    print(f"[PYTHON] EEG automatic output mode = {auto_output_mode}")
                    print_help(auto_output_mode)

                # ---- Quit
                elif key == b'q':
                    print("Exiting...")
                    break

            # ======================================================
            # 2) NON-BLOCKING EEG READ
            # ======================================================
            data_buffer = pull_available_eeg(inlet, data_buffer, max_samples=64)

            # ======================================================
            # 3) EEG PROCESSING / AUTO CLASSIFICATION
            # ======================================================
            now = time.time()
            if eeg_auto_enabled and (now - last_process_time >= PROCESS_INTERVAL_SEC):
                last_process_time = now

                # silent focus / warmup window after pressing E
                if now < decision_ready_at:
                    continue

                current_mu_power = get_mu_power_motor(data_buffer)
                erd_ratio = (
                    current_mu_power / baseline_power
                    if baseline_power and baseline_power > 0
                    else np.nan
                )

                result = classify_current_window(data_buffer, rest_vec, class1_info, class2_info)

                print(
                    f"Mu(C3,Cz,C4): {current_mu_power:.6f} | "
                    f"ratio: {erd_ratio:.3f} | "
                    f"restDist: {result['feat_rest_dist']:.3f} | "
                    f"d_{CLASS1_NAME}: {result['d1']:.3f} | "
                    f"d_{CLASS2_NAME}: {result['d2']:.3f} | "
                    f"best: {result['best_label']} | "
                    f"margin: {result['margin']:.3f} | "
                    f"valid: {result['valid']}"
                )

                if result["valid"] and not robot_state["busy"] and now > robot_busy_until:
                    if consecutive_label == result["best_label"]:
                        consecutive_count += 1
                    else:
                        consecutive_label = result["best_label"]
                        consecutive_count = 1
                else:
                    consecutive_label = None
                    consecutive_count = 0

                if (
                    consecutive_count >= REQUIRED_CONSECUTIVE
                    and not robot_state["busy"]
                    and now > robot_busy_until
                ):
                    label = result["best_label"]
                    print(f"[EEG] {label} confirmed ({consecutive_count} windows) -> sending route ({auto_output_mode})")
                    send_route_for_label(ser, label, auto_output_mode)

                    # after one EEG-driven route, disable EEG auto mode
                    eeg_auto_enabled = False
                    consecutive_label = None
                    consecutive_count = 0
                    robot_busy_until = now + 2.0

                    print("[PYTHON] EEG automatic route selection = OFF")
                    print("[PYTHON] Press E again when you want a new EEG choice.")

            time.sleep(0.005)

    except KeyboardInterrupt:
        print("\nStopped by user.")

    finally:
        ser.close()
        print("Arduino serial closed.")


if __name__ == "__main__":
    main()