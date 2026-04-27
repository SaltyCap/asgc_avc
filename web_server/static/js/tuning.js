const pollIntervalMs = 600;
let tuningState = null;

function redirectToLogin() {
    const next = `${window.location.pathname}${window.location.search}`;
    window.location.href = `/login?next=${encodeURIComponent(next)}`;
}

function redirectToSetup() {
    const next = `${window.location.pathname}${window.location.search}`;
    window.location.href = `/setup?next=${encodeURIComponent(next)}`;
}

async function fetchJson(url, options = {}) {
    const response = await fetch(url, options);
    if (response.status === 401) {
        redirectToLogin();
        throw new Error("Unauthorized");
    }
    if (response.status === 503) {
        let payload = {};
        try {
            payload = await response.json();
        } catch (_) {
        }
        if (payload.setup_required) {
            redirectToSetup();
        }
        throw new Error(payload.error || "Service unavailable");
    }
    const payload = await response.json();
    if (!response.ok) {
        throw new Error(payload.error || "Request failed");
    }
    return payload;
}

function valueIfNotFocused(el, value) {
    if (!el || document.activeElement === el || value === undefined || value === null) {
        return;
    }
    el.value = String(value);
}

function updateStatusCard(state) {
    tuningState = state;

    const controllerMode = document.getElementById("controllerMode");
    const activeTest = document.getElementById("activeTest");
    const phaseText = document.getElementById("phaseText");
    const resultText = document.getElementById("resultText");
    const errorRow = document.getElementById("errorRow");
    const errorText = document.getElementById("errorText");
    const stopTestBtn = document.getElementById("stopTestBtn");

    const mode = (state.nav_controller_mode || "pid").toLowerCase();
    controllerMode.textContent = mode;
    activeTest.textContent = state.active_test || "idle";
    phaseText.textContent = state.phase || "idle";
    resultText.textContent = state.last_result || "none";

    const autotuneStateText = document.getElementById("autotuneStateText");
    const autotuneCostText = document.getElementById("autotuneCostText");
    autotuneStateText.textContent = state.autotune_state || "idle";
    autotuneCostText.textContent = state.autotune_best_cost ? state.autotune_best_cost.toFixed(2) + "s" : "-";

    if (state.last_error) {
        errorRow.style.display = "flex";
        errorText.textContent = state.last_error;
    } else {
        errorRow.style.display = "none";
        errorText.textContent = "none";
    }

    stopTestBtn.disabled = !state.running;

    const turn = state.profiles.turn || {};
    valueIfNotFocused(document.getElementById("turnKp"), turn.kp);
    valueIfNotFocused(document.getElementById("turnKi"), turn.ki);
    valueIfNotFocused(document.getElementById("turnKd"), turn.kd_velocity);
    valueIfNotFocused(document.getElementById("turnKa"), turn.ka_accel);
    valueIfNotFocused(document.getElementById("turnVstop"), turn.velocity_stop_threshold);

    const drive = state.profiles.drive || {};
    valueIfNotFocused(document.getElementById("driveKp"), drive.kp);
    valueIfNotFocused(document.getElementById("driveKi"), drive.ki);
    valueIfNotFocused(document.getElementById("driveKd"), drive.kd_velocity);
    valueIfNotFocused(document.getElementById("driveKa"), drive.ka_accel);
    valueIfNotFocused(document.getElementById("driveVstop"), drive.velocity_stop_threshold);

    const runTurnBtn = document.getElementById("runTurnBtn");
    const runDriveBtn = document.getElementById("runDriveBtn");
    const setPidModeBtn = document.getElementById("setPidModeBtn");
    const setMlModeBtn = document.getElementById("setMlModeBtn");
    const autotuneTurnBtn = document.getElementById("autotuneTurnBtn");
    const autotuneDriveBtn = document.getElementById("autotuneDriveBtn");
    const pauseAutotuneBtn = document.getElementById("pauseAutotuneBtn");
    const cancelAutotuneBtn = document.getElementById("cancelAutotuneBtn");

    const isRunningSomething = !!state.running || (state.autotune_state && state.autotune_state !== "idle" && state.autotune_state !== "complete" && state.autotune_state !== "failed");
    runTurnBtn.disabled = isRunningSomething;
    runDriveBtn.disabled = isRunningSomething;
    autotuneTurnBtn.disabled = isRunningSomething;
    autotuneDriveBtn.disabled = isRunningSomething;
    setPidModeBtn.disabled = isRunningSomething || mode === "pid";
    setMlModeBtn.disabled = isRunningSomething || mode === "ml";

    if (state.autotune_state === "running" || state.autotune_state === "starting") {
        pauseAutotuneBtn.disabled = false;
        pauseAutotuneBtn.textContent = "Pause Autotune";
    } else if (state.autotune_state === "paused") {
        pauseAutotuneBtn.disabled = false;
        pauseAutotuneBtn.textContent = "Resume Autotune";
    } else {
        pauseAutotuneBtn.disabled = true;
        pauseAutotuneBtn.textContent = "Pause Autotune";
    }
    
    cancelAutotuneBtn.disabled = !(state.autotune_state && state.autotune_state !== "idle" && state.autotune_state !== "complete" && state.autotune_state !== "failed" && state.autotune_state !== "cancelling");
}

function gatherProfile(prefix) {
    return {
        kp: parseFloat(document.getElementById(`${prefix}Kp`).value),
        ki: parseFloat(document.getElementById(`${prefix}Ki`).value),
        kd_velocity: parseFloat(document.getElementById(`${prefix}Kd`).value),
        ka_accel: parseFloat(document.getElementById(`${prefix}Ka`).value),
        velocity_stop_threshold: parseFloat(document.getElementById(`${prefix}Vstop`).value),
    };
}

async function refreshStatus() {
    try {
        const state = await fetchJson("/api/tuning/status");
        updateStatusCard(state);
    } catch (err) {
        console.error("Failed to fetch tuning status:", err.message);
    }
}

async function applyProfile(profile, prefix) {
    const gains = gatherProfile(prefix);
    const body = { profile, ...gains };
    const state = await fetchJson("/api/tuning/pid", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
    });
    updateStatusCard(state);
}

async function setControllerMode(mode) {
    const state = await fetchJson("/api/tuning/controller", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode }),
    });
    updateStatusCard(state);
}

async function saveProfile() {
    const state = await fetchJson("/api/tuning/profile/save", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
    });
    updateStatusCard(state);
}

async function loadProfile() {
    const state = await fetchJson("/api/tuning/profile/load", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
    });
    updateStatusCard(state);
}

async function runTurningTest() {
    const angle = parseFloat(document.getElementById("turnAngle").value || "90");
    const state = await fetchJson("/api/tuning/tests/turning", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ angle_deg: angle }),
    });
    updateStatusCard(state);
}

async function runStraightTest() {
    const state = await fetchJson("/api/tuning/tests/straight", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
    });
    updateStatusCard(state);
}

async function stopTest() {
    const state = await fetchJson("/api/tuning/tests/stop", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
    });
    updateStatusCard(state);
}

async function startAutotune(profile) {
    const state = await fetchJson("/api/tuning/autotune/start", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ profile: profile }),
    });
    updateStatusCard(state);
}

async function pauseOrResumeAutotune() {
    if (tuningState && tuningState.autotune_state === "paused") {
        const state = await fetchJson("/api/tuning/autotune/resume", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
        });
        updateStatusCard(state);
    } else {
        const state = await fetchJson("/api/tuning/autotune/pause", {
            method: "POST",
            headers: { "Content-Type": "application/json" },
        });
        updateStatusCard(state);
    }
}

async function cancelAutotune() {
    const state = await fetchJson("/api/tuning/autotune/cancel", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
    });
    updateStatusCard(state);
}

// ===== PWM Slider =====
const PWM_LIMIT_KEY = 'asgc_pwm_limit';
const PW_NEUTRAL = 1500;
const PW_FORWARD_MAX = 2000;
const PW_REVERSE_MAX = 1000;

let pwmMotorWs = null;
let pwmMotorAuthFailed = false;

function applyPwmLimit(value) {
    const slider = document.getElementById('pwmLimitSlider');
    const pctLabel = document.getElementById('pwmLimitValue');
    const minEl = document.getElementById('pwMinValue');
    const maxEl = document.getElementById('pwMaxValue');
    if (!slider) return;
    slider.value = value;
    pctLabel.textContent = `${value}%`;
    minEl.textContent = Math.round(PW_NEUTRAL - (PW_NEUTRAL - PW_REVERSE_MAX) * value / 100);
    maxEl.textContent = Math.round(PW_NEUTRAL + (PW_FORWARD_MAX - PW_NEUTRAL) * value / 100);
}

function sendPwmToServer(value) {
    if (pwmMotorWs && pwmMotorWs.readyState === WebSocket.OPEN) {
        pwmMotorWs.send(JSON.stringify({ type: 'set_pwm', min_pwm_percent: value, max_pwm_percent: value }));
        pwmMotorWs.send(JSON.stringify({ type: 'set_speed', speed_percent: value }));
    }
}

function connectPwmWebSocket() {
    const connDot = document.getElementById('pwmConnStatus');
    const connText = document.getElementById('pwmConnText');
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    pwmMotorWs = new WebSocket(`${proto}//${window.location.host}/motor`);

    pwmMotorWs.onopen = () => {
        if (connDot) connDot.classList.add('connected');
        if (connText) connText.textContent = 'Connected';
        const saved = parseInt(localStorage.getItem(PWM_LIMIT_KEY) ?? '25', 10);
        sendPwmToServer(saved);
    };

    pwmMotorWs.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'pwm_set') {
                const calculated = Math.round(data.max_pwm_percent ?? 25);
                const current = parseInt(localStorage.getItem(PWM_LIMIT_KEY) ?? '25', 10);
                if (calculated !== current) {
                    localStorage.setItem(PWM_LIMIT_KEY, calculated);
                    applyPwmLimit(calculated);
                }
            }
        } catch (e) { /* ignore */ }
    };

    pwmMotorWs.onclose = () => {
        if (connDot) connDot.classList.remove('connected');
        if (connText) connText.textContent = 'Disconnected';
        if (!pwmMotorAuthFailed) setTimeout(connectPwmWebSocket, 3000);
    };

    pwmMotorWs.onerror = () => {
        if (connDot) connDot.classList.remove('connected');
        if (connText) connText.textContent = 'Error';
    };
}

// ===== Telemetry Graph =====
let telemetryWs = null;
let telemetryChart = null;

function initTelemetryChart() {
    const ctx = document.getElementById('telemetryChart');
    if (!ctx) return;
    
    telemetryChart = new Chart(ctx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'Actual Heading',
                    data: [],
                    borderColor: 'rgba(54, 162, 235, 1)',
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 2,
                    tension: 0.1,
                    fill: false,
                    pointRadius: 0
                },
                {
                    label: 'Target Heading',
                    data: [],
                    borderColor: 'rgba(255, 99, 132, 1)',
                    borderDash: [5, 5],
                    borderWidth: 2,
                    tension: 0,
                    fill: false,
                    pointRadius: 0
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            scales: {
                x: {
                    display: false // hide time labels to keep it clean
                },
                y: {
                    title: {
                        display: true,
                        text: 'Heading (deg)'
                    }
                }
            },
            plugins: {
                legend: {
                    position: 'top',
                }
            }
        }
    });
}

function connectTelemetryWebSocket() {
    const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    telemetryWs = new WebSocket(`${proto}//${window.location.host}/tuning_telemetry`);

    let lastAddedHeading = null;

    telemetryWs.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'telemetry' && telemetryChart) {
                const maxPoints = 150; // 15 seconds at 10Hz
                const timeStr = new Date(data.time * 1000).toLocaleTimeString();
                
                let h = data.heading;
                if (lastAddedHeading !== null) {
                    while (h - lastAddedHeading > 180) h -= 360;
                    while (h - lastAddedHeading < -180) h += 360;
                }
                lastAddedHeading = h;

                let th = data.target_heading;
                if (th !== null) {
                    while (th - h > 180) th -= 360;
                    while (th - h < -180) th += 360;
                }
                
                telemetryChart.data.labels.push(timeStr);
                telemetryChart.data.datasets[0].data.push(h);
                telemetryChart.data.datasets[1].data.push(th !== null ? th : null);
                
                if (telemetryChart.data.labels.length > maxPoints) {
                    telemetryChart.data.labels.shift();
                    telemetryChart.data.datasets[0].data.shift();
                    telemetryChart.data.datasets[1].data.shift();
                }
                
                telemetryChart.update();
            }
        } catch (e) { /* ignore */ }
    };

    telemetryWs.onclose = () => {
        setTimeout(connectTelemetryWebSocket, 3000);
    };
}

document.addEventListener("DOMContentLoaded", () => {
    const saveTurnBtn = document.getElementById("saveTurnBtn");
    const runTurnBtn = document.getElementById("runTurnBtn");
    const saveDriveBtn = document.getElementById("saveDriveBtn");
    const runDriveBtn = document.getElementById("runDriveBtn");
    const stopTestBtn = document.getElementById("stopTestBtn");
    const setPidModeBtn = document.getElementById("setPidModeBtn");
    const setMlModeBtn = document.getElementById("setMlModeBtn");
    const saveProfileBtn = document.getElementById("saveProfileBtn");
    const loadProfileBtn = document.getElementById("loadProfileBtn");
    const autotuneTurnBtn = document.getElementById("autotuneTurnBtn");
    const autotuneDriveBtn = document.getElementById("autotuneDriveBtn");
    const pauseAutotuneBtn = document.getElementById("pauseAutotuneBtn");
    const cancelAutotuneBtn = document.getElementById("cancelAutotuneBtn");

    saveTurnBtn.addEventListener("click", async () => {
        try {
            await applyProfile("turn", "turn");
        } catch (err) {
            alert(`Failed to save turning PID: ${err.message}`);
        }
    });

    runTurnBtn.addEventListener("click", async () => {
        try {
            await runTurningTest();
        } catch (err) {
            alert(`Failed to start turning test: ${err.message}`);
        }
    });

    saveDriveBtn.addEventListener("click", async () => {
        try {
            await applyProfile("drive", "drive");
        } catch (err) {
            alert(`Failed to save straight PID: ${err.message}`);
        }
    });

    runDriveBtn.addEventListener("click", async () => {
        try {
            await runStraightTest();
        } catch (err) {
            alert(`Failed to start straight test: ${err.message}`);
        }
    });

    stopTestBtn.addEventListener("click", async () => {
        try {
            await stopTest();
        } catch (err) {
            alert(`Failed to stop test: ${err.message}`);
        }
    });

    setPidModeBtn.addEventListener("click", async () => {
        try {
            await setControllerMode("pid");
        } catch (err) {
            alert(`Failed to set PID mode: ${err.message}`);
        }
    });

    setMlModeBtn.addEventListener("click", async () => {
        try {
            await setControllerMode("ml");
        } catch (err) {
            alert(`Failed to set ML mode: ${err.message}`);
        }
    });

    saveProfileBtn.addEventListener("click", async () => {
        try {
            await saveProfile();
        } catch (err) {
            alert(`Failed to save profile: ${err.message}`);
        }
    });

    loadProfileBtn.addEventListener("click", async () => {
        try {
            await loadProfile();
        } catch (err) {
            alert(`Failed to load profile: ${err.message}`);
        }
    });

    autotuneTurnBtn.addEventListener("click", async () => {
        try {
            await startAutotune("turn");
        } catch (err) {
            alert(`Failed to start turning autotune: ${err.message}`);
        }
    });

    autotuneDriveBtn.addEventListener("click", async () => {
        try {
            await startAutotune("drive");
        } catch (err) {
            alert(`Failed to start straight autotune: ${err.message}`);
        }
    });

    pauseAutotuneBtn.addEventListener("click", async () => {
        try {
            await pauseOrResumeAutotune();
        } catch (err) {
            alert(`Failed to pause/resume autotune: ${err.message}`);
        }
    });

    cancelAutotuneBtn.addEventListener("click", async () => {
        try {
            await cancelAutotune();
        } catch (err) {
            alert(`Failed to cancel autotune: ${err.message}`);
        }
    });

    // PWM slider initialization
    const pwmLimitSlider = document.getElementById('pwmLimitSlider');
    if (pwmLimitSlider) {
        const saved = parseInt(localStorage.getItem(PWM_LIMIT_KEY) ?? '25', 10);
        applyPwmLimit(saved);
        pwmLimitSlider.addEventListener('input', (e) => {
            const value = parseInt(e.target.value, 10);
            localStorage.setItem(PWM_LIMIT_KEY, value);
            applyPwmLimit(value);
            sendPwmToServer(value);
        });
    }

    // Cross-tab PWM sync
    window.addEventListener('storage', (e) => {
        if (e.key === PWM_LIMIT_KEY && e.newValue !== null) {
            const newLimit = parseInt(e.newValue, 10);
            const current = parseInt(localStorage.getItem(PWM_LIMIT_KEY) ?? '25', 10);
            if (newLimit !== current) {
                localStorage.setItem(PWM_LIMIT_KEY, newLimit);
                applyPwmLimit(newLimit);
                sendPwmToServer(newLimit);
            }
        }
    });

    initTelemetryChart();
    connectTelemetryWebSocket();
    connectPwmWebSocket();
    refreshStatus();
    setInterval(refreshStatus, pollIntervalMs);
});
