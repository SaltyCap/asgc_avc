// Car Control Variables
const carStartButton = document.getElementById('carStartButton');
const carStopButton = document.getElementById('carStopButton');
const carStatusText = document.getElementById('carStatusText');
const carStatusDot = document.getElementById('carStatusDot');
let carRunning = false;

// Voice Control Variables
const voiceStartButton = document.getElementById('voiceStartButton');
const voiceStopButton = document.getElementById('voiceStopButton');
const voiceStatusText = document.getElementById('voiceStatusText');
const voiceStatusDot = document.getElementById('voiceStatusDot');
const transcriptEl = document.getElementById('transcript');
let ws;
let finalTranscript = '';
let audioContext;
let source;
let processor;

// Queue Control Variables
const queueStatus = document.getElementById('queueStatus');
const clearQueueBtn = document.getElementById('clearQueueBtn');
const resetPosBtn = document.getElementById('resetPosBtn');
const queueSlots = [
    document.getElementById('slot0'),
    document.getElementById('slot1'),
    document.getElementById('slot2'),
    document.getElementById('slot3'),
    document.getElementById('slot4')
];
let motorWs;

// PWM Limit Control Variables (also controls speed/throttle)
const pwmLimitSlider = document.getElementById('pwmLimitSlider');
const pwmLimitValue = document.getElementById('pwmLimitValue');
const pwmMinValue = document.getElementById('pwMinValue');
const pwmMaxValue = document.getElementById('pwMaxValue');
let pwmLimit = 25; // 0-100%, scales the PWM output range

// PWM pulse width constants (in microseconds)
const PW_NEUTRAL = 1500;
const PW_FORWARD_START = 1550;  // ESC deadband: forward starts here
const PW_FORWARD_MAX = 2000;
const PW_REVERSE_START = 1450;  // ESC deadband: reverse starts here
const PW_REVERSE_MAX = 1000;

// Connection Status Variables
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');

// ===== CAR CONTROL FUNCTIONS =====

/**
 * Starts the car navigation queue.
 */
function startCar() {
    carRunning = true;
    carStartButton.disabled = true;
    carStopButton.disabled = false;
    carStatusText.textContent = 'Queue: Running';
    carStatusDot.classList.add('running');

    console.log('Queue started!');
    // Start the command queue
    startQueue();
}

/**
 * Stops the car and clears the navigation queue.
 */
function stopCar() {
    carRunning = false;
    carStartButton.disabled = false;
    carStopButton.disabled = true;
    carStatusText.textContent = 'Queue: Stopped';
    carStatusDot.classList.remove('running');

    console.log('Queue stopped!');
    // Stop the command queue
    stopQueue();
}

// ===== VOICE CONTROL FUNCTIONS =====

/**
 * Updates the UI state for voice recording.
 * @param {boolean} isRecording - Whether recording is active
 */
function setVoiceUIState(isRecording) {
    voiceStartButton.disabled = isRecording;
    voiceStopButton.disabled = !isRecording;
    if (isRecording) {
        voiceStopButton.classList.add('recording-pulse');
        voiceStatusText.textContent = 'Recording...';
        voiceStatusDot.classList.add('recording');
        voiceStatusDot.classList.remove('active');
    } else {
        voiceStopButton.classList.remove('recording-pulse');
        voiceStatusText.textContent = 'Ready';
        voiceStatusDot.classList.remove('recording');
        voiceStatusDot.classList.add('active');
    }
}

function connectWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/audio`;
    ws = new WebSocket(wsUrl);
    ws.binaryType = 'arraybuffer';

    ws.onopen = () => {
        voiceStatusText.textContent = 'Ready';
        voiceStatusDot.classList.add('active');
        voiceStartButton.disabled = false;
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);
        if (data.type === 'partial') {
            transcriptEl.textContent = finalTranscript + data.text;
        } else if (data.type === 'final') {
            finalTranscript += data.text + ' ';
            transcriptEl.textContent = finalTranscript;
        }
    };

    ws.onclose = () => {
        voiceStatusText.textContent = 'Disconnected';
        voiceStatusDot.classList.remove('active', 'recording');
        voiceStartButton.disabled = true;
        voiceStopButton.disabled = true;
        if (processor) {
            processor.disconnect();
            processor = null;
        }
        if (source) {
            source.disconnect();
            source = null;
        }
        if (audioContext) {
            audioContext.close();
            audioContext = null;
        }
        // Auto-reconnect
        setTimeout(connectWebSocket, 3000);
    };

    ws.onerror = (error) => {
        console.error('WebSocket Error:', error);
        voiceStatusText.textContent = 'Connection Error';
        voiceStatusDot.classList.remove('active', 'recording');
        voiceStartButton.disabled = true;
        voiceStopButton.disabled = true;
    };
}

function floatTo16BitPCM(float32Array) {
    const buffer = new ArrayBuffer(float32Array.length * 2);
    const view = new DataView(buffer);
    let offset = 0;
    for (let i = 0; i < float32Array.length; i++, offset += 2) {
        const s = Math.max(-1, Math.min(1, float32Array[i]));
        view.setInt16(offset, s < 0 ? s * 0x8000 : s * 0x7FFF, true);
    }
    return buffer;
}

async function startVoiceRecording() {
    if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
        voiceStatusText.textContent = 'Microphone not supported';
        return;
    }
    finalTranscript = '';
    transcriptEl.textContent = 'Listening...';

    try {
        const stream = await navigator.mediaDevices.getUserMedia({
            audio: {
                sampleRate: 16000,
                channelCount: 1,
                echoCancellation: true,
                noiseSuppression: true,
                autoGainControl: true,
                googEchoCancellation: true,
                googAutoGainControl: true,
                googNoiseSuppression: true,
                googHighpassFilter: true
            }
        });

        audioContext = new (window.AudioContext || window.webkitAudioContext)({ sampleRate: 16000 });
        source = audioContext.createMediaStreamSource(stream);
        processor = audioContext.createScriptProcessor(2048, 1, 1);

        processor.onaudioprocess = (e) => {
            if (ws && ws.readyState === WebSocket.OPEN) {
                const float32Array = e.inputBuffer.getChannelData(0);
                const int16Array = floatTo16BitPCM(float32Array);
                ws.send(int16Array);
            }
        };

        source.connect(processor);
        processor.connect(audioContext.destination);

        if (ws && ws.readyState === WebSocket.OPEN) {
            ws.send('start');
            setVoiceUIState(true);
        }
    } catch (err) {
        console.error('Error accessing microphone:', err);
        voiceStatusText.textContent = `Error: ${err.message}`;
    }
}

function stopVoiceRecording() {
    if (processor) {
        processor.disconnect();
        processor = null;
    }
    if (source) {
        source.disconnect();
        source = null;
    }
    if (audioContext) {
        audioContext.close();
        audioContext = null;
    }
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send('stop');
    }
    setVoiceUIState(false);
}

// ===== QUEUE CONTROL FUNCTIONS =====
function connectMotorWebSocket() {
    const wsProtocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${wsProtocol}//${window.location.host}/motor`;
    motorWs = new WebSocket(wsUrl);

    motorWs.onopen = () => {
        console.log('Motor WebSocket connected');
        if (statusDot) statusDot.classList.add('connected');
        if (statusText) statusText.textContent = 'Connected';
        // Send current PWM limit (also sends speed/throttle)
        updatePwmLimitDisplay();
    };

    motorWs.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);

            // Handle PWM settings broadcast from server
            if (data.type === 'pwm_set') {
                // Update slider to match the new PWM limit
                // Reverse-calculate limit from min_pwm and max_pwm
                const receivedMinPwm = data.min_pwm || 45;
                const receivedMaxPwm = data.max_pwm || 100;

                // Calculate the limit percentage from max_pwm (assuming BASE_MAX_PWM = 100)
                const calculatedLimit = Math.round((receivedMaxPwm / 100) * 100);

                // Update slider without triggering a send (avoid feedback loop)
                if (pwmLimitSlider && calculatedLimit !== pwmLimit) {
                    pwmLimit = calculatedLimit;
                    pwmLimitSlider.value = calculatedLimit;
                    pwmLimitValue.textContent = `${calculatedLimit}%`;

                    // Update pulse width display
                    const minPwUs = Math.round(PW_NEUTRAL - (PW_NEUTRAL - PW_REVERSE_MAX) * pwmLimit / 100);
                    const maxPwUs = Math.round(PW_NEUTRAL + (PW_FORWARD_MAX - PW_NEUTRAL) * pwmLimit / 100);
                    pwMinValue.textContent = minPwUs;
                    pwMaxValue.textContent = maxPwUs;

                    console.log(`PWM limit synchronized to ${calculatedLimit}%`);
                }
            }
        } catch (e) {
            console.error('Error parsing motor WebSocket message:', e);
        }
    };

    motorWs.onclose = () => {
        console.log('Motor WebSocket disconnected');
        if (statusDot) statusDot.classList.remove('connected');
        if (statusText) statusText.textContent = 'Disconnected';
        setTimeout(connectMotorWebSocket, 3000);
    };

    motorWs.onerror = (error) => {
        console.error('Motor WebSocket error:', error);
        if (statusDot) statusDot.classList.remove('connected');
        if (statusText) statusText.textContent = 'Error';
    };
}

/**
 * Sends a command string to the motor control WebSocket.
 * @param {string} command - The command to send
 */
function sendQueueCommand(command) {
    if (motorWs && motorWs.readyState === WebSocket.OPEN) {
        motorWs.send(JSON.stringify({ type: 'voice', command: command }));
    }
}

function startQueue() {
    sendQueueCommand('start');
}

function stopQueue() {
    sendQueueCommand('stop');
}

function clearQueue() {
    sendQueueCommand('clear');
}

function resetPosition() {
    sendQueueCommand('reset position');
}

function updatePwmLimitDisplay() {
    if (!pwmLimitSlider) return; // Guard against missing element

    pwmLimit = parseInt(pwmLimitSlider.value);
    pwmLimitValue.textContent = `${pwmLimit}%`;

    // Calculate the limited pulse width range based on slider
    // At 0%: min = max = neutral (1500)
    // At 100%: min = 1000, max = 2000 (full range)
    // Scale proportionally around neutral, accounting for ESC deadband
    const minPwUs = Math.round(PW_NEUTRAL - (PW_NEUTRAL - PW_REVERSE_MAX) * pwmLimit / 100);
    const maxPwUs = Math.round(PW_NEUTRAL + (PW_FORWARD_MAX - PW_NEUTRAL) * pwmLimit / 100);

    // Display pulse widths
    pwMinValue.textContent = minPwUs;
    pwMaxValue.textContent = maxPwUs;

    // Convert pulse widths to PWM percentages for the C program
    // The C program uses MIN_PWM and MAX_PWM as percentages (0-100)
    // We want to map the active range to these percentages
    // At 100% limit: min_pwm should be ~45%, max_pwm should be ~100%
    // At 50% limit: range is narrower, so both values scale down

    // Map pulse width range to PWM percent:
    // Full reverse (1000us) to full forward (2000us) = 0% to 100% of available power
    // Accounting for deadband: 1450-1550us is neutral zone

    // For reverse: 1000-1450us maps to 0-45% PWM (minimum to overcome friction)
    // For forward: 1550-2000us maps to 45-100% PWM

    // Calculate effective min/max PWM percentages based on limit
    // Base values: min=45%, max=100%
    const BASE_MIN_PWM = 45;
    const BASE_MAX_PWM = 100;

    // Scale PWM values by limit percentage
    // At 0% limit: both should be ~0 (no movement)
    // At 100% limit: use base values
    let minPwm = Math.round(BASE_MIN_PWM * pwmLimit / 100);
    let maxPwm = Math.round(BASE_MAX_PWM * pwmLimit / 100);

    // Ensure minimum is at least 20% when limit > 0, to overcome friction
    if (pwmLimit > 0 && minPwm < 20) {
        minPwm = 20;
    }

    sendPwmSettings(minPwm, maxPwm);
}

function sendPwmSettings(minPwm, maxPwm) {
    if (motorWs && motorWs.readyState === WebSocket.OPEN) {
        // Send PWM limits (which now also controls speed)
        motorWs.send(JSON.stringify({
            type: 'set_pwm',
            min_pwm: minPwm,
            max_pwm: maxPwm
        }));

        console.log(`Power Limit: ${pwmLimit}% (PWM: Min=${minPwm}%, Max=${maxPwm}%)`);
    }
}

function fetchQueueStatus() {
    fetch('/api/navigation/status')
        .then(response => response.json())
        .then(data => {
            if (!data.error) {
                updateQueueDisplay(data.queue || [], data.queue_running, data.target);
            }
        })
        .catch(() => { });
}

function updateQueueDisplay(queue, isRunning, currentTarget) {
    // Update status indicator
    if (isRunning) {
        queueStatus.textContent = 'Running';
        queueStatus.classList.add('running');
        queueStatus.classList.remove('waiting');
    } else if (queue.length === 0 && !currentTarget) {
        queueStatus.textContent = 'Waiting For Next Command';
        queueStatus.classList.remove('running');
        queueStatus.classList.add('waiting');
    } else {
        queueStatus.textContent = 'Ready';
        queueStatus.classList.remove('running', 'waiting');
    }

    // Build combined list: current target (if running) + queue
    const allCommands = [];

    if (isRunning && currentTarget) {
        const targetName = getTargetName(currentTarget);
        allCommands.push({ target: targetName, active: true });
    }

    // Add queued commands
    queue.forEach(cmd => {
        allCommands.push({ target: cmd.target, active: false });
    });

    // Update the 5 fixed slots
    for (let i = 0; i < 5; i++) {
        const slot = queueSlots[i];
        const label = slot.querySelector('.slot-label');

        // Remove all color classes
        slot.classList.remove('red', 'yellow', 'blue', 'green', 'center', 'filled', 'active');

        if (i < allCommands.length) {
            const cmd = allCommands[i];
            const colorClass = cmd.target.toLowerCase();

            slot.classList.add(colorClass, 'filled');
            if (cmd.active) {
                slot.classList.add('active');
            }
            label.textContent = cmd.target;
        } else {
            label.textContent = '---';
        }
    }
}

function getTargetName(target) {
    if (!target) return '';
    if (Array.isArray(target)) {
        const [x, y] = target;
        if (x === 15 && y === 15) return 'CENTER';
        if (x === 0 && y === 0) return 'RED';
        if (x === 0 && y === 30) return 'YELLOW';
        if (x === 30 && y === 30) return 'BLUE';
        if (x === 30 && y === 0) return 'GREEN';
        return `(${x}, ${y})`;
    }
    return String(target);
}

// ===== INITIALIZATION =====
document.addEventListener('DOMContentLoaded', () => {
    // Car control initialization
    carStartButton.addEventListener('click', startCar);
    carStopButton.addEventListener('click', stopCar);
    carStopButton.disabled = true;

    // Voice control initialization
    voiceStartButton.disabled = true;
    voiceStopButton.disabled = true;
    voiceStartButton.addEventListener('click', startVoiceRecording);
    voiceStopButton.addEventListener('click', stopVoiceRecording);
    connectWebSocket();

    // Queue control initialization
    clearQueueBtn.addEventListener('click', clearQueue);
    resetPosBtn.addEventListener('click', resetPosition);

    // Power limit slider initialization
    if (pwmLimitSlider) {
        pwmLimitSlider.addEventListener('input', updatePwmLimitDisplay);
    }

    connectMotorWebSocket();
    fetchQueueStatus();
    setInterval(fetchQueueStatus, 500);
});
