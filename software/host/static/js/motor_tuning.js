(function () {
    const body = document.body;
    const motorId = Number(body.dataset.motorId || "1");

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

    function outputRatio() {
        return Number(document.getElementById("outputRatio").value || "10");
    }

    function timeoutSeconds() {
        return Number(document.getElementById("timeoutValue").value || "0.2");
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

    async function refreshStatus() {
        try {
            const payload = await apiFetch(`/api/motor_tuning/status?output_ratio=${encodeURIComponent(outputRatio())}`);
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
            resultPanel.textContent = formatJson(payload);
            await refreshStatus();
        } catch (error) {
            resultPanel.textContent = `Read failed (${action}):\n${error.message}`;
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
            resultPanel.textContent = formatJson(payload);
            await refreshStatus();
        } catch (error) {
            resultPanel.textContent = `Runtime action failed (${action}):\n${error.message}`;
        }
    }

    document.getElementById("refreshStatusBtn").addEventListener("click", refreshStatus);

    document.querySelectorAll(".tuning-read").forEach((button) => {
        button.addEventListener("click", () => performRead(button.dataset.read));
    });

    document.querySelectorAll(".runtime-action").forEach((button) => {
        button.addEventListener("click", () => performRuntimeAction(button.dataset.action));
    });

    refreshStatus();
})();
