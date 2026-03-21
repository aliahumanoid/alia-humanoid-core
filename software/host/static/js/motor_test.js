(function () {
    const body = document.body;
    const motorId = Number(body.dataset.motorId || "1");
    let selectedPresetId = null;

    const SWEEP_PRESETS = [
        {
            id: "torque-step-plus25",
            title: "Torque Step +25",
            category: "Threshold",
            summary: "Positive breakaway check at the practical minimum motion threshold.",
            note: "Use to confirm the first sustained motion region on CAN without stalling below threshold.",
            suggestedLabel: "preset_torque_step_p25",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: 25,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-step-minus25",
            title: "Torque Step -25",
            category: "Threshold",
            summary: "Negative breakaway check at the practical minimum motion threshold.",
            note: "Useful to verify the low-torque asymmetry without using values that usually fail to move.",
            suggestedLabel: "preset_torque_step_m25",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: -25,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-step-plus50",
            title: "Torque Step +50",
            category: "Compare",
            summary: "Direct RS485/CAN comparison point on the positive branch.",
            note: "This is the cleanest step used in the validation notes.",
            suggestedLabel: "preset_torque_step_p50",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: 50,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-step-minus50",
            title: "Torque Step -50",
            category: "Compare",
            summary: "Direct RS485/CAN comparison point on the negative branch.",
            note: "Use together with +50 for symmetry checks.",
            suggestedLabel: "preset_torque_step_m50",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: -50,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-chirp-plus25",
            title: "Torque Chirp +25 / 5",
            category: "Bandwidth",
            summary: "Positive torque chirp used for the corrected contiguous-band result.",
            note: "Reference preset for the ~7 Hz small-signal torque-mode validation.",
            suggestedLabel: "preset_torque_chirp_p25_a5",
            values: {
                sweepMode: "torque",
                sweepProfile: "chirp",
                sweepDuration: 12,
                sweepRate: 100,
                timeoutValue: 0.05,
                sweepBias: 25,
                sweepAmplitude: 5,
                sweepPreload: 0,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-chirp-minus25",
            title: "Torque Chirp -25 / 5",
            category: "Bandwidth",
            summary: "Negative torque chirp used for the corrected contiguous-band result.",
            note: "Pairs with the positive chirp to check branch asymmetry.",
            suggestedLabel: "preset_torque_chirp_m25_a5",
            values: {
                sweepMode: "torque",
                sweepProfile: "chirp",
                sweepDuration: 12,
                sweepRate: 100,
                timeoutValue: 0.05,
                sweepBias: -25,
                sweepAmplitude: 5,
                sweepPreload: 0,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "speed-chirp-plus3000",
            title: "Speed Chirp +3000 / 800",
            category: "Bandwidth",
            summary: "Positive speed-mode chirp used to cross-check torque-mode behavior.",
            note: "Reference preset for the speed-loop comparison branch.",
            suggestedLabel: "preset_speed_chirp_p3000_a800",
            values: {
                sweepMode: "speed",
                sweepProfile: "chirp",
                sweepDuration: 12,
                sweepRate: 100,
                timeoutValue: 0.05,
                sweepBias: 3000,
                sweepAmplitude: 800,
                sweepPreload: 0,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 6,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "speed-chirp-minus3000",
            title: "Speed Chirp -3000 / 800",
            category: "Bandwidth",
            summary: "Negative speed-mode chirp used to compare against the positive branch.",
            note: "Use this after the positive speed chirp for direct symmetry checks.",
            suggestedLabel: "preset_speed_chirp_m3000_a800",
            values: {
                sweepMode: "speed",
                sweepProfile: "chirp",
                sweepDuration: 12,
                sweepRate: 100,
                timeoutValue: 0.05,
                sweepBias: -3000,
                sweepAmplitude: 800,
                sweepPreload: 0,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 6,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-step-plus300",
            title: "Torque Step +300",
            category: "Higher Load",
            summary: "Large-signal positive torque step for tomorrow's higher-command check.",
            note: "Use only once low-torque checks are complete and the bench is clear.",
            suggestedLabel: "preset_torque_step_p300",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: 300,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
        {
            id: "torque-step-minus300",
            title: "Torque Step -300",
            category: "Higher Load",
            summary: "Large-signal negative torque step for tomorrow's higher-command check.",
            note: "Use after confirming the positive branch remains stable.",
            suggestedLabel: "preset_torque_step_m300",
            values: {
                sweepMode: "torque",
                sweepProfile: "step",
                sweepDuration: 3,
                sweepRate: 50,
                timeoutValue: 0.1,
                sweepBias: 0,
                sweepAmplitude: -300,
                sweepPreload: 1,
                sweepFrequency: 1,
                sweepF0: 0.2,
                sweepF1: 8,
                stopAtEnd: true,
                motorOnBefore: true,
                powerOffAtEnd: false,
                extraSingle: false,
                extraMulti: false,
            },
        },
    ];

    const statusDot = document.getElementById("statusDot");
    const statusText = document.getElementById("statusText");
    const statusInterface = document.getElementById("statusInterface");
    const statusChannel = document.getElementById("statusChannel");
    const statusLastRx = document.getElementById("statusLastRx");
    const snapshotPanel = document.getElementById("snapshotPanel");
    const resultPanel = document.getElementById("resultPanel");
    const logList = document.getElementById("logList");
    const analysisFilename = document.getElementById("analysisFilename");
    const analysisSummary = document.getElementById("analysisSummary");
    const analysisPlotSpeed = document.getElementById("analysisPlotSpeed");
    const analysisPlotPosition = document.getElementById("analysisPlotPosition");
    const presetList = document.getElementById("presetList");
    const presetSummary = document.getElementById("presetSummary");
    const clearPresetBtn = document.getElementById("clearPresetBtn");

    const sweepFieldIds = [
        "sweepMode",
        "sweepProfile",
        "sweepDuration",
        "sweepRate",
        "timeoutValue",
        "sweepBias",
        "sweepAmplitude",
        "sweepPreload",
        "sweepFrequency",
        "sweepF0",
        "sweepF1",
        "sweepLabel",
        "stopAtEnd",
        "motorOnBefore",
        "powerOffAtEnd",
        "extraRaw",
        "extraSingle",
        "extraMulti",
    ];
    const sweepFields = Object.fromEntries(
        sweepFieldIds.map((id) => [id, document.getElementById(id)]),
    );

    function outputRatio() {
        return Number(document.getElementById("outputRatio").value || "10");
    }

    function timeoutSeconds() {
        return Number(document.getElementById("timeoutValue").value || "0.1");
    }

    function formatTimestamp(epochSeconds) {
        if (!epochSeconds) {
            return "-";
        }
        return new Date(epochSeconds * 1000).toLocaleString();
    }

    function renderJson(target, data) {
        target.textContent = JSON.stringify(data, null, 2);
    }

    function setFieldValue(element, value) {
        if (!element) {
            return;
        }
        if (element.type === "checkbox") {
            element.checked = Boolean(value);
            return;
        }
        element.value = value;
    }

    function findPreset(presetId) {
        return SWEEP_PRESETS.find((preset) => preset.id === presetId) || null;
    }

    function updateProfileFieldState() {
        const profile = sweepFields.sweepProfile.value;
        const groups = {
            sweepPreload: profile === "step",
            sweepFrequency: profile === "square",
            sweepF0: profile === "chirp",
            sweepF1: profile === "chirp",
        };

        Object.entries(groups).forEach(([id, enabled]) => {
            const field = sweepFields[id];
            if (!field) {
                return;
            }
            field.disabled = !enabled;
            field.classList.toggle("opacity-50", !enabled);
        });
    }

    function renderPresetSummary(preset) {
        if (!preset) {
            presetSummary.textContent = "No preset selected. Manual edits are still allowed.";
            return;
        }

        const values = preset.values;
        presetSummary.innerHTML = `
            <div class="flex flex-wrap items-center gap-2 mb-2">
                <span class="pill">${preset.category}</span>
                <span class="mono text-sky-300">${preset.suggestedLabel}</span>
            </div>
            <div class="font-semibold text-white">${preset.title}</div>
            <div class="mt-1 text-slate-300">${preset.summary}</div>
            <div class="mt-2 text-slate-400">${preset.note}</div>
            <div class="mt-3 mono text-xs text-slate-400">
                mode=${values.sweepMode} profile=${values.sweepProfile} bias=${values.sweepBias}
                amp=${values.sweepAmplitude} duration=${values.sweepDuration}s rate=${values.sweepRate}Hz
                timeout=${values.timeoutValue}s f0=${values.sweepF0}Hz f1=${values.sweepF1}Hz
            </div>
        `;
    }

    function renderPresets() {
        presetList.innerHTML = SWEEP_PRESETS.map((preset) => {
            const selected = preset.id === selectedPresetId;
            const classes = selected
                ? "border-sky-400/80 bg-sky-500/12 ring-1 ring-sky-400/35"
                : "border-slate-700/70 bg-slate-950/25";
            return `
                <button type="button"
                    class="text-left rounded-2xl border px-4 py-3 transition-colors hover:border-sky-300/50 hover:bg-sky-500/8 ${classes}"
                    data-preset-id="${preset.id}">
                    <div class="flex items-center justify-between gap-3">
                        <div class="font-semibold text-white">${preset.title}</div>
                        <span class="pill">${preset.category}</span>
                    </div>
                    <div class="mt-2 text-sm text-slate-300">${preset.summary}</div>
                    <div class="mt-2 mono text-xs text-slate-400">${preset.suggestedLabel}</div>
                </button>
            `;
        }).join("");

        presetList.querySelectorAll("[data-preset-id]").forEach((button) => {
            button.addEventListener("click", () => {
                applyPreset(button.dataset.presetId);
            });
        });
    }

    function applyPreset(presetId) {
        const preset = findPreset(presetId);
        if (!preset) {
            return;
        }
        selectedPresetId = preset.id;
        Object.entries({ extraRaw: false, ...preset.values }).forEach(([id, value]) => {
            setFieldValue(sweepFields[id], value);
        });
        setFieldValue(sweepFields.sweepLabel, preset.suggestedLabel);
        updateProfileFieldState();
        renderPresets();
        renderPresetSummary(preset);
    }

    function clearPresetSelection() {
        selectedPresetId = null;
        renderPresets();
        renderPresetSummary(null);
    }

    async function requestJson(url, options) {
        const response = await fetch(url, options);
        const payload = await response.json();
        if (!response.ok || payload.status === "error") {
            const error = new Error(payload.message || `Request failed: ${response.status}`);
            error.payload = payload;
            throw error;
        }
        return payload;
    }

    function setConnectionUi(connected) {
        statusDot.classList.toggle("connected", connected);
        statusDot.classList.toggle("disconnected", !connected);
        statusText.textContent = connected ? "Connected" : "Disconnected";
    }

    function renderLogs(logs) {
        if (!logs || logs.length === 0) {
            logList.innerHTML = '<div class="text-slate-500">No CSV logs yet.</div>';
            return;
        }

        logList.innerHTML = logs.map((log) => {
            const encoded = encodeURIComponent(log.name);
            const url = `/api/motor_test/logs/${encoded}`;
            return `
                <div class="rounded-xl border border-slate-700/70 bg-slate-950/25 px-4 py-3">
                    <div class="mono text-sky-300">${log.name}</div>
                    <div class="text-xs text-slate-400 mt-1">
                        ${Math.round((log.size_bytes || 0) / 1024)} KB - ${formatTimestamp(log.modified_ts)}
                    </div>
                    <div class="flex gap-2 mt-3">
                        <button class="btn btn-secondary text-xs log-analyze" data-filename="${log.name}">Analyze</button>
                        <a href="${url}" class="btn btn-secondary text-xs">Download</a>
                        <button class="btn btn-danger text-xs log-delete" data-filename="${log.name}">Delete</button>
                    </div>
                </div>
            `;
        }).join("");

        document.querySelectorAll(".log-analyze").forEach((button) => {
            button.addEventListener("click", async () => {
                try {
                    await loadAnalysis(button.dataset.filename);
                } catch (error) {
                    renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
                }
            });
        });

        document.querySelectorAll(".log-delete").forEach((button) => {
            button.addEventListener("click", async () => {
                if (!window.confirm(`Delete ${button.dataset.filename}?`)) {
                    return;
                }
                try {
                    const payload = await requestJson(`/api/motor_test/logs/${encodeURIComponent(button.dataset.filename)}`, {
                        method: "DELETE",
                    });
                    renderJson(resultPanel, payload);
                    await loadStatus();
                } catch (error) {
                    renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
                }
            });
        });
    }

    async function loadAnalysis(filename) {
        const payload = await requestJson(`/api/motor_test/logs/${encodeURIComponent(filename)}/analysis`);
        analysisFilename.textContent = filename;
        renderJson(analysisSummary, {
            metadata: payload.metadata,
            summary: payload.summary,
            row_count: payload.row_count,
        });

        const t = payload.series.t_s || [];
        Plotly.newPlot(analysisPlotSpeed, [
            {
                x: t,
                y: payload.series.target || [],
                name: "target",
                mode: "lines",
                line: { color: "#38bdf8", width: 2 },
                yaxis: "y1",
            },
            {
                x: t,
                y: payload.series.speed_dps || [],
                name: "speed_dps",
                mode: "lines",
                line: { color: "#22c55e", width: 2 },
                yaxis: "y2",
            },
            {
                x: t,
                y: payload.series.iq_counts || [],
                name: "iq_counts",
                mode: "lines",
                line: { color: "#f59e0b", width: 1.5 },
                yaxis: "y3",
            },
        ], {
            title: "Target / Speed / IQ",
            paper_bgcolor: "rgba(0,0,0,0)",
            plot_bgcolor: "rgba(0,0,0,0)",
            font: { color: "#edf4ff" },
            margin: { t: 40, r: 60, b: 40, l: 60 },
            xaxis: { title: "Time [s]", gridcolor: "rgba(148,163,184,0.15)" },
            yaxis: { title: "Target", gridcolor: "rgba(148,163,184,0.15)" },
            yaxis2: { title: "Speed [dps]", overlaying: "y", side: "right" },
            yaxis3: { title: "IQ", overlaying: "y", side: "right", anchor: "free", position: 0.92 },
            legend: { orientation: "h" },
        }, { responsive: true });

        Plotly.newPlot(analysisPlotPosition, [
            {
                x: t,
                y: payload.series.actuator_abs_deg || [],
                name: "actuator_abs_deg",
                mode: "lines",
                line: { color: "#a78bfa", width: 2 },
            },
        ], {
            title: "Actuator Absolute Angle",
            paper_bgcolor: "rgba(0,0,0,0)",
            plot_bgcolor: "rgba(0,0,0,0)",
            font: { color: "#edf4ff" },
            margin: { t: 40, r: 40, b: 40, l: 60 },
            xaxis: { title: "Time [s]", gridcolor: "rgba(148,163,184,0.15)" },
            yaxis: { title: "Angle [deg]", gridcolor: "rgba(148,163,184,0.15)" },
        }, { responsive: true });
    }

    async function loadStatus() {
        const payload = await requestJson(`/api/motor_test/status?output_ratio=${encodeURIComponent(outputRatio())}`);
        setConnectionUi(Boolean(payload.connected));
        statusInterface.textContent = payload.config?.interface || "-";
        statusChannel.textContent = payload.config?.channel || "-";
        statusLastRx.textContent = formatTimestamp(payload.last_rx_timestamp);
        renderJson(snapshotPanel, {
            motor_id: motorId,
            output_ratio: payload.output_ratio,
            config: payload.config,
            motor_recent: payload.motor_recent,
            stats: payload.stats,
        });
        renderLogs(payload.recent_logs || []);
        return payload;
    }

    async function runAction(action, value) {
        const body = {
            action,
            timeout_s: timeoutSeconds(),
            output_ratio: outputRatio(),
        };
        if (value !== undefined) {
            body.value = value;
        }
        const payload = await requestJson("/api/motor_test/action", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(body),
        });
        renderJson(resultPanel, payload);
        await loadStatus();
    }

    async function runSweep() {
        const extraCommands = [];
        if (document.getElementById("extraRaw").checked) {
            extraCommands.push("raw");
        }
        if (document.getElementById("extraSingle").checked) {
            extraCommands.push("single");
        }
        if (document.getElementById("extraMulti").checked) {
            extraCommands.push("multi");
        }

        const response = await fetch("/api/motor_test/sweep", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({
                mode: document.getElementById("sweepMode").value,
                profile: document.getElementById("sweepProfile").value,
                duration_s: Number(document.getElementById("sweepDuration").value || "3"),
                rate_hz: Number(document.getElementById("sweepRate").value || "50"),
                timeout_s: timeoutSeconds(),
                output_ratio: outputRatio(),
                bias: Number(document.getElementById("sweepBias").value || "0"),
                amplitude: Number(document.getElementById("sweepAmplitude").value || "0"),
                preload_s: Number(document.getElementById("sweepPreload").value || "0"),
                frequency_hz: Number(document.getElementById("sweepFrequency").value || "1"),
                f0_hz: Number(document.getElementById("sweepF0").value || "0.2"),
                f1_hz: Number(document.getElementById("sweepF1").value || "8"),
                label: document.getElementById("sweepLabel").value,
                extra_commands: extraCommands,
                stop_at_end: document.getElementById("stopAtEnd").checked,
                motor_on_before: document.getElementById("motorOnBefore").checked,
                power_off_at_end: document.getElementById("powerOffAtEnd").checked,
            }),
        });
        const payload = await response.json();
        renderJson(resultPanel, payload);
        await loadStatus();
        if (payload.csv_name) {
            await loadAnalysis(payload.csv_name);
        }
        if (!response.ok || payload.status === "error") {
            const error = new Error(payload.message || `Request failed: ${response.status}`);
            error.payload = payload;
            throw error;
        }
        return payload;
    }

    function bindEvents() {
        renderPresets();
        renderPresetSummary(null);
        updateProfileFieldState();

        document.getElementById("refreshStatusBtn").addEventListener("click", async () => {
            try {
                await loadStatus();
            } catch (error) {
                renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
            }
        });

        clearPresetBtn.addEventListener("click", () => {
            clearPresetSelection();
        });

        sweepFields.sweepProfile.addEventListener("change", () => {
            updateProfileFieldState();
        });

        document.querySelectorAll(".motor-action").forEach((button) => {
            button.addEventListener("click", async () => {
                try {
                    await runAction(button.dataset.action);
                } catch (error) {
                    renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
                }
            });
        });

        document.getElementById("sendTorqueBtn").addEventListener("click", async () => {
            try {
                await runAction("torque", Number(document.getElementById("torqueValue").value || "0"));
            } catch (error) {
                renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
            }
        });

        document.getElementById("sendSpeedBtn").addEventListener("click", async () => {
            try {
                await runAction("speed", Number(document.getElementById("speedValue").value || "0"));
            } catch (error) {
                renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
            }
        });

        document.getElementById("runSweepBtn").addEventListener("click", async () => {
            try {
                renderJson(resultPanel, { status: "running", message: "Sweep in progress..." });
                await runSweep();
            } catch (error) {
                renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
            }
        });

        document.getElementById("loadLogsBtn").addEventListener("click", async () => {
            try {
                const payload = await requestJson("/api/motor_test/logs");
                renderLogs(payload.logs || []);
            } catch (error) {
                renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
            }
        });
    }

    bindEvents();
    loadStatus().catch((error) => {
        renderJson(resultPanel, { status: "error", message: error.message, payload: error.payload });
    });
    window.setInterval(() => {
        loadStatus().catch(() => {
            // Keep polling quiet when the CAN bus is temporarily unavailable.
        });
    }, 3000);
})();
