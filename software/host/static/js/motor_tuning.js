(function () {
    const body = document.body;
    const motorId = Number(body.dataset.motorId || "1");
    const rs485MotorId = Number(body.dataset.rs485MotorId || "1");
    let latestCanStatus = null;
    let latestPrivateStatus = null;
    let latestResult = null;

    const statusDot = document.getElementById("statusDot");
    const statusText = document.getElementById("statusText");
    const statusInterface = document.getElementById("statusInterface");
    const statusChannel = document.getElementById("statusChannel");
    const statusLastRx = document.getElementById("statusLastRx");
    const resultPanel = document.getElementById("resultPanel");

    const pidPosition = document.getElementById("pidPosition");
    const pidSpeed = document.getElementById("pidSpeed");
    const pidTorque = document.getElementById("pidTorque");
    const accelerationPanel = document.getElementById("accelerationPanel");

    const snapshotState2 = document.getElementById("snapshotState2");
    const snapshotSingle = document.getElementById("snapshotSingle");
    const snapshotMulti = document.getElementById("snapshotMulti");
    const privatePort = document.getElementById("privatePort");
    const privateBaud = document.getElementById("privateBaud");
    const privateCurrentPid = document.getElementById("privateCurrentPid");
    const privateLimits = document.getElementById("privateLimits");
    const privateCalib = document.getElementById("privateCalib");
    const readFullBaselineBtn = document.getElementById("readFullBaselineBtn");
    const exportBaselineBtn = document.getElementById("exportBaselineBtn");
    const loadDraftFromLatestBtn = document.getElementById("loadDraftFromLatestBtn");
    const restoreCurrentPidBtn = document.getElementById("restoreCurrentPidBtn");
    const exportDraftBtn = document.getElementById("exportDraftBtn");
    const confirmCurrentPidWrite = document.getElementById("confirmCurrentPidWrite");
    const applyCurrentPidRamBtn = document.getElementById("applyCurrentPidRamBtn");
    const draftSummary = document.getElementById("draftSummary");
    let draftInitialized = false;

    function outputRatio() {
        return Number(document.getElementById("outputRatio").value || "10");
    }

    function timeoutSeconds() {
        return Number(document.getElementById("timeoutValue").value || "0.2");
    }

    function privateTimeoutSeconds() {
        return Math.max(timeoutSeconds(), 0.5);
    }

    function formatTimestamp(epochSeconds) {
        if (!epochSeconds) {
            return "-";
        }
        return new Date(epochSeconds * 1000).toLocaleString();
    }

    function formatJson(value) {
        if (value === null || value === undefined) {
            return "-";
        }
        return JSON.stringify(value, null, 2);
    }

    function timestampTag(date = new Date()) {
        const pad = (value) => String(value).padStart(2, "0");
        return [
            date.getFullYear(),
            pad(date.getMonth() + 1),
            pad(date.getDate()),
            "_",
            pad(date.getHours()),
            pad(date.getMinutes()),
            pad(date.getSeconds()),
        ].join("");
    }

    function downloadJsonFile(filename, payload) {
        const blob = new Blob([JSON.stringify(payload, null, 2)], { type: "application/json" });
        const url = URL.createObjectURL(blob);
        const anchor = document.createElement("a");
        anchor.href = url;
        anchor.download = filename;
        anchor.click();
        URL.revokeObjectURL(url);
    }

    async function apiFetch(url, options = {}) {
        const response = await fetch(url, {
            headers: { "Content-Type": "application/json" },
            ...options,
        });
        const payload = await response.json();
        if (!response.ok || payload.status === "error") {
            throw new Error(payload.message || `Request failed (${response.status})`);
        }
        return payload;
    }

    function draftFieldRefs() {
        return {
            angleKp: document.getElementById("draftAngleKp"),
            angleKi: document.getElementById("draftAngleKi"),
            angleKd: document.getElementById("draftAngleKd"),
            speedKp: document.getElementById("draftSpeedKp"),
            speedKi: document.getElementById("draftSpeedKi"),
            speedKd: document.getElementById("draftSpeedKd"),
            currentKp: document.getElementById("draftCurrentKp"),
            currentKi: document.getElementById("draftCurrentKi"),
            currentKd: document.getElementById("draftCurrentKd"),
            maxTorque: document.getElementById("draftMaxTorque"),
            currentRamp: document.getElementById("draftCurrentRamp"),
            speedRamp: document.getElementById("draftSpeedRamp"),
            maxSpeed: document.getElementById("draftMaxSpeed"),
            maxAngle: document.getElementById("draftMaxAngle"),
        };
    }

    function draftPayloadFromInputs() {
        const refs = draftFieldRefs();
        const read = (element) => {
            if (!element || element.value === "") {
                return null;
            }
            return Number(element.value);
        };
        return {
            motor_id: motorId,
            rs485_motor_id: rs485MotorId,
            source: "vendor_private_setting_0x14",
            draft: {
                angle_pid: {
                    kp: read(refs.angleKp),
                    ki: read(refs.angleKi),
                    kd: read(refs.angleKd),
                },
                speed_pid: {
                    kp: read(refs.speedKp),
                    ki: read(refs.speedKi),
                    kd: read(refs.speedKd),
                },
                current_pid: {
                    kp: read(refs.currentKp),
                    ki: read(refs.currentKi),
                    kd: read(refs.currentKd),
                },
                max_torque_current_counts: read(refs.maxTorque),
                current_ramp: read(refs.currentRamp),
                speed_ramp: read(refs.speedRamp),
                max_speed_dps: read(refs.maxSpeed),
                max_angle_deg: read(refs.maxAngle),
            },
            note: "Draft only. No RAM/ROM write has been sent.",
        };
    }

    function latestSettingDecoded() {
        return latestPrivateStatus?.recent?.setting?.decoded || null;
    }

    function latestSettingRawHex() {
        return latestSettingDecoded()?._raw_hex || null;
    }

    function renderDraftSummary() {
        draftSummary.textContent = formatJson(draftPayloadFromInputs());
    }

    function populateDraftFromSetting(decoded, force = false) {
        if (!decoded) {
            return;
        }
        if (draftInitialized && !force) {
            return;
        }
        const refs = draftFieldRefs();
        refs.angleKp.value = decoded.anglePidKp ?? "";
        refs.angleKi.value = decoded.anglePidKi ?? "";
        refs.angleKd.value = decoded.anglePidKd ?? "";
        refs.speedKp.value = decoded.speedPidKp ?? "";
        refs.speedKi.value = decoded.speedPidKi ?? "";
        refs.speedKd.value = decoded.speedPidKd ?? "";
        refs.currentKp.value = decoded.currentPidKp ?? "";
        refs.currentKi.value = decoded.currentPidKi ?? "";
        refs.currentKd.value = decoded.currentPidKd ?? "";
        refs.maxTorque.value = decoded.maxTorque ?? "";
        refs.currentRamp.value = decoded.currentRamp ?? "";
        refs.speedRamp.value = decoded.speedRamp ?? "";
        refs.maxSpeed.value = decoded.maxSpeed__scaled ?? "";
        refs.maxAngle.value = decoded.maxAngle__scaled ?? "";
        draftInitialized = true;
        renderDraftSummary();
    }

    function renderPid(pid) {
        if (!pid) {
            pidPosition.textContent = "-";
            pidSpeed.textContent = "-";
            pidTorque.textContent = "-";
            return;
        }
        pidPosition.textContent = `Kp ${pid.position_pid_kp} / Ki ${pid.position_pid_ki}`;
        pidSpeed.textContent = `Kp ${pid.speed_pid_kp} / Ki ${pid.speed_pid_ki}`;
        pidTorque.textContent = `Kp ${pid.torque_pid_kp} / Ki ${pid.torque_pid_ki}`;
    }

    function renderAcceleration(accel) {
        if (!accel) {
            accelerationPanel.textContent = "-";
            return;
        }
        accelerationPanel.textContent = `${accel.acceleration_dps2}`;
    }

    function renderRecent(recent) {
        renderPid(recent?.pid || null);
        renderAcceleration(recent?.acceleration || null);
        snapshotState2.textContent = formatJson(recent?.state2 || null);
        snapshotSingle.textContent = formatJson(recent?.single || null);
        snapshotMulti.textContent = formatJson(recent?.multi || null);
    }

    function renderPrivateRecent(recent) {
        const settingDecoded = recent?.setting?.decoded || null;
        const settingSummary = recent?.setting?.decoded?._summary || null;
        const calibSummary = recent?.calib || null;

        if (settingSummary) {
            privateCurrentPid.textContent = formatJson(settingSummary.current_pid || null);
            privateLimits.textContent = formatJson({
                max_torque_current_counts: settingSummary.max_torque_current_counts,
                current_ramp: settingSummary.current_ramp,
                speed_ramp: settingSummary.speed_ramp,
                max_speed_dps: settingSummary.max_speed_dps,
                max_angle_deg: settingSummary.max_angle_deg,
            });
        } else {
            privateCurrentPid.textContent = "-";
            privateLimits.textContent = "-";
        }

        if (calibSummary) {
            privateCalib.textContent = formatJson({
                data_len: calibSummary.data_len,
                data_hex: calibSummary.data_hex,
                note: calibSummary.note,
            });
        } else {
            privateCalib.textContent = "-";
        }

        if (settingDecoded) {
            populateDraftFromSetting(settingDecoded);
        }
        updateCurrentPidWriteEnabled();
    }

    function renderPrivatePorts(ports) {
        const previous = privatePort.value;
        privatePort.innerHTML = "";
        if (!ports || !ports.length) {
            const option = document.createElement("option");
            option.value = "";
            option.textContent = "No serial ports found";
            privatePort.appendChild(option);
            return;
        }
        ports.forEach((port) => {
            const option = document.createElement("option");
            option.value = port;
            option.textContent = port;
            privatePort.appendChild(option);
        });
        if (ports.includes(previous)) {
            privatePort.value = previous;
        }
    }

    async function refreshStatus() {
        try {
            const payload = await apiFetch(`/api/motor_tuning/status?output_ratio=${encodeURIComponent(outputRatio())}`);
            latestCanStatus = payload;
            const config = payload.config || {};
            statusDot.classList.toggle("connected", !!payload.connected);
            statusDot.classList.toggle("disconnected", !payload.connected);
            statusText.textContent = payload.connected ? "Connected" : "Disconnected";
            statusInterface.textContent = config.interface || "-";
            statusChannel.textContent = config.channel || "-";
            statusLastRx.textContent = payload.last_rx_timestamp ? formatTimestamp(payload.last_rx_timestamp) : "-";
            renderRecent(payload.recent || {});
        } catch (error) {
            resultPanel.textContent = `Status refresh failed:\n${error.message}`;
        }
    }

    async function refreshPrivateStatus() {
        try {
            const payload = await apiFetch("/api/motor_tuning/private_status");
            latestPrivateStatus = payload;
            renderPrivatePorts(payload.available_ports || []);
            if (payload.baud && !privateBaud.value) {
                privateBaud.value = String(payload.baud);
            }
            renderPrivateRecent(payload.recent || {});
        } catch (error) {
            resultPanel.textContent = `Private status refresh failed:\n${error.message}`;
        }
    }

    async function performRead(action) {
        try {
            const payload = await apiFetch("/api/motor_tuning/read", {
                method: "POST",
                body: JSON.stringify({
                    action,
                    timeout_s: timeoutSeconds(),
                    output_ratio: outputRatio(),
                }),
            });
            latestResult = payload;
            resultPanel.textContent = formatJson(payload);
            await refreshStatus();
            return payload;
        } catch (error) {
            resultPanel.textContent = `Read failed (${action}):\n${error.message}`;
            return null;
        }
    }

    async function performRuntimeAction(action) {
        try {
            const payload = await apiFetch("/api/motor_test/action", {
                method: "POST",
                body: JSON.stringify({
                    action,
                    timeout_s: timeoutSeconds(),
                    output_ratio: outputRatio(),
                }),
            });
            latestResult = payload;
            resultPanel.textContent = formatJson(payload);
            await refreshStatus();
            return payload;
        } catch (error) {
            resultPanel.textContent = `Runtime action failed (${action}):\n${error.message}`;
            return null;
        }
    }

    async function performPrivateRead(action) {
        try {
            const payload = await apiFetch("/api/motor_tuning/private_read", {
                method: "POST",
                body: JSON.stringify({
                    action,
                    port: privatePort.value,
                    baud: Number(privateBaud.value || "115200"),
                    timeout_s: privateTimeoutSeconds(),
                    motor_id: rs485MotorId,
                }),
            });
            latestResult = payload;
            resultPanel.textContent = formatJson(payload);
            await refreshPrivateStatus();
            return payload;
        } catch (error) {
            resultPanel.textContent = `Private read failed (${action}):\n${error.message}`;
            return null;
        }
    }

    async function readFullBaseline() {
        readFullBaselineBtn.disabled = true;
        exportBaselineBtn.disabled = true;
        const originalLabel = readFullBaselineBtn.textContent;
        readFullBaselineBtn.textContent = "Reading...";
        try {
            if (!await performPrivateRead("setting")) {
                throw new Error("private setting read failed");
            }
            if (!await performPrivateRead("calib")) {
                throw new Error("private calib read failed");
            }
            const runtimeFailures = [];
            if (!await performRead("state2")) {
                runtimeFailures.push("state2");
            }
            if (!await performRead("single")) {
                runtimeFailures.push("single");
            }
            if (!await performRead("multi")) {
                runtimeFailures.push("multi");
            }
            resultPanel.textContent = `Full baseline refresh completed at ${new Date().toLocaleString()}.\n\n` +
                `Private setting/calib are now cached and ready for export.` +
                (runtimeFailures.length
                    ? `\nRuntime CAN reads unavailable in this pass: ${runtimeFailures.join(", ")}.`
                    : `\nRuntime state2/single/multi are also cached.`);
        } catch (error) {
            resultPanel.textContent = `Full baseline refresh failed:\n${error.message}`;
        } finally {
            readFullBaselineBtn.disabled = false;
            exportBaselineBtn.disabled = false;
            readFullBaselineBtn.textContent = originalLabel;
        }
    }

    function exportBaselineSnapshot() {
        if (!latestCanStatus && !latestPrivateStatus) {
            resultPanel.textContent = "Export failed:\nNo baseline data available yet. Read the private setting or run Read Full Baseline first.";
            return;
        }

        const payload = {
            exported_at: new Date().toISOString(),
            motor_id: motorId,
            rs485_motor_id: rs485MotorId,
            output_ratio: outputRatio(),
            rs485_port: privatePort.value || null,
            rs485_baud: Number(privateBaud.value || "115200"),
            can_status: latestCanStatus,
            private_status: latestPrivateStatus,
            last_response: latestResult,
            note: "Read-only baseline snapshot captured from /motor_tuning before any RAM/ROM writes.",
        };
        const filename = `motor${motorId}_baseline_${timestampTag()}.json`;
        downloadJsonFile(filename, payload);
        resultPanel.textContent = `Baseline snapshot exported as ${filename}.`;
    }

    function loadDraftFromLatestSetting() {
        const decoded = latestSettingDecoded();
        if (!decoded) {
            resultPanel.textContent = "Load draft failed:\nRead Private Setting (0x14) first so the form has a baseline to copy.";
            return;
        }
        populateDraftFromSetting(decoded, true);
        resultPanel.textContent = "Draft form loaded from the latest private setting snapshot.";
    }

    function restoreCurrentPidFromLatestSetting() {
        const decoded = latestSettingDecoded();
        if (!decoded) {
            resultPanel.textContent = "Restore current PID failed:\nRead Private Setting (0x14) first.";
            return;
        }
        const refs = draftFieldRefs();
        refs.currentKp.value = decoded.currentPidKp ?? "";
        refs.currentKi.value = decoded.currentPidKi ?? "";
        refs.currentKd.value = decoded.currentPidKd ?? "";
        draftInitialized = true;
        renderDraftSummary();
        resultPanel.textContent = "Current PID fields restored from the latest private setting snapshot.";
    }

    function exportDraftSnapshot() {
        if (!draftInitialized) {
            resultPanel.textContent = "Export draft failed:\nLoad the latest private setting into the draft form first.";
            return;
        }
        const payload = {
            exported_at: new Date().toISOString(),
            baseline_source: latestPrivateStatus?.recent?.setting || null,
            ...draftPayloadFromInputs(),
        };
        const filename = `motor${motorId}_tuning_draft_${timestampTag()}.json`;
        downloadJsonFile(filename, payload);
        resultPanel.textContent = `Tuning draft exported as ${filename}.`;
    }

    function updateCurrentPidWriteEnabled() {
        const hasBase = !!latestSettingRawHex();
        const confirmed = !!confirmCurrentPidWrite?.checked;
        if (applyCurrentPidRamBtn) {
            applyCurrentPidRamBtn.disabled = !(hasBase && confirmed);
            applyCurrentPidRamBtn.classList.toggle("opacity-50", applyCurrentPidRamBtn.disabled);
            applyCurrentPidRamBtn.classList.toggle("cursor-not-allowed", applyCurrentPidRamBtn.disabled);
        }
    }

    async function applyCurrentPidRam() {
        const baseRawHex = latestSettingRawHex();
        if (!baseRawHex) {
            resultPanel.textContent = "Current PID write failed:\nRead Private Setting (0x14) first so we have the base blob to patch.";
            return;
        }
        if (!confirmCurrentPidWrite.checked) {
            resultPanel.textContent = "Current PID write blocked:\nTick the confirmation checkbox first.";
            return;
        }

        const draft = draftPayloadFromInputs().draft;
        applyCurrentPidRamBtn.disabled = true;
        const originalLabel = applyCurrentPidRamBtn.textContent;
        applyCurrentPidRamBtn.textContent = "Writing...";
        try {
            const payload = await apiFetch("/api/motor_tuning/private_write_ram", {
                method: "POST",
                body: JSON.stringify({
                    action: "current_pid_ram",
                    port: privatePort.value,
                    baud: Number(privateBaud.value || "115200"),
                    timeout_s: privateTimeoutSeconds(),
                    base_raw_hex: baseRawHex,
                    current_pid: draft.current_pid,
                    confirm_token: "WRITE_CURRENT_PID_RAM",
                }),
            });
            latestResult = payload;
            resultPanel.textContent = formatJson(payload);
            await refreshPrivateStatus();
        } catch (error) {
            resultPanel.textContent = `Current PID write failed:\n${error.message}`;
        } finally {
            applyCurrentPidRamBtn.textContent = originalLabel;
            updateCurrentPidWriteEnabled();
        }
    }

    document.getElementById("refreshStatusBtn").addEventListener("click", refreshStatus);
    document.getElementById("refreshPrivatePortsBtn").addEventListener("click", refreshPrivateStatus);
    readFullBaselineBtn.addEventListener("click", readFullBaseline);
    exportBaselineBtn.addEventListener("click", exportBaselineSnapshot);
    loadDraftFromLatestBtn.addEventListener("click", loadDraftFromLatestSetting);
    restoreCurrentPidBtn.addEventListener("click", restoreCurrentPidFromLatestSetting);
    exportDraftBtn.addEventListener("click", exportDraftSnapshot);
    confirmCurrentPidWrite.addEventListener("change", updateCurrentPidWriteEnabled);
    applyCurrentPidRamBtn.addEventListener("click", applyCurrentPidRam);

    Object.values(draftFieldRefs()).forEach((element) => {
        element.addEventListener("input", () => {
            draftInitialized = true;
            renderDraftSummary();
        });
    });

    document.querySelectorAll(".tuning-read").forEach((button) => {
        button.addEventListener("click", () => performRead(button.dataset.read));
    });

    document.querySelectorAll(".runtime-action").forEach((button) => {
        button.addEventListener("click", () => performRuntimeAction(button.dataset.action));
    });

    document.querySelectorAll(".private-read").forEach((button) => {
        button.addEventListener("click", () => performPrivateRead(button.dataset.privateRead));
    });

    refreshStatus();
    refreshPrivateStatus();
    renderDraftSummary();
    updateCurrentPidWriteEnabled();
})();
