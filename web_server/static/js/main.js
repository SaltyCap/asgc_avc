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

// Throttle Control Variables
const throttleSlider = document.getElementById('throttleSlider');
const throttleValue = document.getElementById('throttleValue');
let currentThrottle = 100;

// Connection Status Variables
const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');

// ===== CAR CONTROL FUNCTIONS =====
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
        // Send current throttle setting
        sendThrottleSetting();
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

function updateThrottleDisplay() {
    currentThrottle = parseInt(throttleSlider.value);
    throttleValue.textContent = `${currentThrottle}%`;
    sendThrottleSetting();
}

function sendThrottleSetting() {
    if (motorWs && motorWs.readyState === WebSocket.OPEN) {
        motorWs.send(JSON.stringify({
            type: 'set_speed',
            speed_percent: currentThrottle
        }));
        console.log(`Throttle set to ${currentThrottle}%`);
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
        .catch(() => {});
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

    // Throttle slider initialization
    throttleSlider.addEventListener('input', updateThrottleDisplay);

    connectMotorWebSocket();
    fetchQueueStatus();
    setInterval(fetchQueueStatus, 500);
});
