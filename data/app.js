/**
 * Satellite Tracker Web UI Application
 * Handles WebSocket communication, UI updates, and user interactions
 */

// =============================================================================
// Global State
// =============================================================================
let ws = null;
let reconnectTimer = null;
let status = {
    azimuth: 0,
    elevation: 0,
    targetAz: 0,
    targetEl: 0,
    moving: false,
    mode: 0,
    satellite: '',
    satVisible: false,
    gpsValid: false
};

const JOG_AMOUNT = 5; // degrees
const MODES = ['Idle', 'Manual', 'Auto Tracking', 'External (rotctld)'];

// =============================================================================
// WebSocket Connection
// =============================================================================
function connectWebSocket() {
    const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${location.host}/ws`;
    
    console.log('Connecting to WebSocket:', wsUrl);
    ws = new WebSocket(wsUrl);
    
    ws.onopen = () => {
        console.log('WebSocket connected');
        setConnectionStatus(true);
        clearTimeout(reconnectTimer);
        loadSatellites();
        fetchGPSStatus();
        fetchWiFiStatus();
        fetchSystemInfo();
    };
    
    ws.onclose = () => {
        console.log('WebSocket disconnected');
        setConnectionStatus(false);
        scheduleReconnect();
    };
    
    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
        setConnectionStatus(false);
    };
    
    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            handleMessage(data);
        } catch (e) {
            console.error('Failed to parse message:', e);
        }
    };
}

function scheduleReconnect() {
    if (reconnectTimer) clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(() => {
        console.log('Attempting reconnect...');
        connectWebSocket();
    }, 3000);
}

function setConnectionStatus(connected) {
    const indicator = document.getElementById('connection-status');
    const text = document.getElementById('connection-text');
    
    if (connected) {
        indicator.className = 'status-dot connected';
        text.textContent = 'Connected';
    } else {
        indicator.className = 'status-dot';
        text.textContent = 'Disconnected';
    }
}

// =============================================================================
// Message Handling
// =============================================================================
function handleMessage(data) {
    if (data.type === 'status') {
        updateStatus(data);
    } else if (data.event) {
        handleEvent(data.event, data.data);
    }
}

function updateStatus(data) {
    status = { ...status, ...data };
    
    // Update position display
    document.getElementById('azimuth').textContent = data.az?.toFixed(1) ?? '---';
    document.getElementById('elevation').textContent = data.el?.toFixed(1) ?? '---';
    
    // Update target display
    if (data.targetAz !== undefined && data.targetEl !== undefined) {
        document.getElementById('target-az').textContent = data.targetAz.toFixed(1);
        document.getElementById('target-el').textContent = data.targetEl.toFixed(1);
        document.getElementById('target-display').style.display = data.moving ? 'block' : 'none';
    }
    
    // Update motion indicator
    document.getElementById('motion-indicator').style.display = data.moving ? 'flex' : 'none';
    
    // Update compass
    updateCompass(data.az ?? 0, data.el ?? 0);
    
    // Update satellite info
    if (data.sat) {
        document.getElementById('sat-name').textContent = data.sat;
        document.getElementById('satellite-info').style.display = 'block';
        
        if (data.satAz !== undefined) {
            document.getElementById('sat-az').textContent = data.satAz.toFixed(1);
            document.getElementById('sat-el').textContent = data.satEl.toFixed(1);
            updateSatelliteMarker(data.satAz, data.satEl, data.visible);
        }
        
        const visEl = document.getElementById('sat-visibility');
        visEl.textContent = data.visible ? 'Above Horizon ✓' : 'Below Horizon';
        visEl.className = 'sat-visibility ' + (data.visible ? 'visible' : 'hidden');
    }
    
    // Update tracking mode
    document.getElementById('tracking-mode').textContent = MODES[data.mode] || 'Unknown';
    
    // Show/hide tracking badge
    const trackingBadge = document.getElementById('tracking-badge');
    if (data.mode === 2) {
        trackingBadge.style.display = 'flex';
    } else {
        trackingBadge.style.display = 'none';
    }
    
    // Update GPS indicator
    const gpsStatus = document.getElementById('gps-status');
    const gpsIcon = document.getElementById('gps-status-icon');
    gpsStatus.textContent = data.gps ? '3D Fix ✓' : 'No Fix';
    gpsStatus.className = 'gps-value ' + (data.gps ? 'fix' : 'nofix');
    gpsIcon.textContent = data.gps ? '✓' : '⏳';
    
    // Update time
    if (data.time) {
        const date = new Date(data.time * 1000);
        document.getElementById('gps-time').textContent = date.toUTCString();
    }
    
    // Update rotctld clients count
    if (data.rotctldClients !== undefined) {
        document.getElementById('sys-rotctld').textContent = data.rotctldClients;
    }
}

function handleEvent(event, data) {
    switch (event) {
        case 'tracking_started':
            showToast('Tracking started: ' + data, 'success');
            updateTrackingUI(true);
            break;
        case 'tracking_stopped':
            showToast('Tracking stopped', 'info');
            updateTrackingUI(false);
            break;
        case 'homing_complete':
            showToast('Homing complete', 'success');
            break;
        case 'error':
            showToast('Error: ' + data, 'error');
            break;
    }
}

// =============================================================================
// Compass Visualization
// =============================================================================
function updateCompass(azimuth, elevation) {
    // Rotate antenna pointer
    const pointer = document.getElementById('antenna-pointer');
    pointer.setAttribute('transform', `rotate(${azimuth}, 100, 100)`);
    
    // Update elevation bar
    const fill = document.getElementById('elevation-fill');
    const percent = (elevation / 90) * 100;
    fill.style.height = `${percent}%`;
}

function updateSatelliteMarker(az, el, visible) {
    const markerGroup = document.getElementById('sat-marker-group');
    const marker = document.getElementById('sat-marker');
    const pulse = document.getElementById('sat-marker-pulse');
    
    if (!visible || el < 0) {
        markerGroup.style.display = 'none';
        return;
    }
    
    markerGroup.style.display = 'block';
    
    // Convert az/el to x,y on compass (el=90 at center, el=0 at edge)
    const radius = 80 * (1 - el / 90);
    const azRad = (az - 90) * Math.PI / 180;
    const x = 100 + radius * Math.cos(azRad);
    const y = 100 + radius * Math.sin(azRad);
    
    marker.setAttribute('cx', x);
    marker.setAttribute('cy', y);
    pulse.setAttribute('cx', x);
    pulse.setAttribute('cy', y);
}

// Initialize compass tick marks
function initCompass() {
    const ticks = document.getElementById('compass-ticks');
    for (let i = 0; i < 360; i += 10) {
        const major = i % 30 === 0;
        const r1 = major ? 82 : 85;
        const r2 = 90;
        const rad = (i - 90) * Math.PI / 180;
        const x1 = 100 + r1 * Math.cos(rad);
        const y1 = 100 + r1 * Math.sin(rad);
        const x2 = 100 + r2 * Math.cos(rad);
        const y2 = 100 + r2 * Math.sin(rad);
        
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        line.setAttribute('x1', x1);
        line.setAttribute('y1', y1);
        line.setAttribute('x2', x2);
        line.setAttribute('y2', y2);
        line.setAttribute('class', major ? 'tick-major' : 'tick-minor');
        ticks.appendChild(line);
    }
}

// =============================================================================
// Manual Control
// =============================================================================
function jogUp() {
    const newEl = Math.min(90, status.elevation + JOG_AMOUNT);
    setPosition(status.azimuth, newEl);
}

function jogDown() {
    const newEl = Math.max(0, status.elevation - JOG_AMOUNT);
    setPosition(status.azimuth, newEl);
}

function jogLeft() {
    setPosition(status.azimuth - JOG_AMOUNT, status.elevation);
}

function jogRight() {
    setPosition(status.azimuth + JOG_AMOUNT, status.elevation);
}

function goToPosition() {
    const az = parseFloat(document.getElementById('goto-az').value);
    const el = parseFloat(document.getElementById('goto-el').value);
    
    if (isNaN(az) || isNaN(el)) {
        showToast('Invalid position', 'error');
        return;
    }
    
    setPosition(az, el);
}

async function setPosition(az, el) {
    try {
        const response = await fetch('/api/position', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ azimuth: az, elevation: el })
        });
        
        if (!response.ok) {
            const data = await response.json();
            showToast(data.error || 'Failed to set position', 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function stopMotion() {
    try {
        await fetch('/api/stop', { method: 'POST' });
        showToast('Motion stopped', 'info');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function parkAntenna() {
    try {
        await fetch('/api/park', { method: 'POST' });
        showToast('Moving to park position', 'info');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function homeAntenna() {
    showToast('Starting homing sequence...', 'info');
    try {
        const response = await fetch('/api/home', { method: 'POST' });
        const data = await response.json();
        showToast(data.message || 'Homing complete', response.ok ? 'success' : 'error');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function setHome() {
    if (!confirm('Set current position as home (0°, 0°)?')) return;
    
    try {
        await fetch('/api/sethome', { method: 'POST' });
        showToast('Home position set', 'success');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

// =============================================================================
// Satellite Tracking
// =============================================================================
async function loadSatellites() {
    try {
        const response = await fetch('/api/tle');
        const data = await response.json();
        
        const select = document.getElementById('satellite-select');
        select.innerHTML = '<option value="">Select Satellite...</option>';
        
        if (data.tles) {
            data.tles.forEach(tle => {
                const option = document.createElement('option');
                option.value = tle.name;
                option.textContent = tle.name + (tle.stale ? ' (stale)' : '');
                select.appendChild(option);
            });
        }
    } catch (e) {
        console.error('Failed to load satellites:', e);
    }
}

async function startTracking() {
    const select = document.getElementById('satellite-select');
    const name = select.value;
    
    if (!name) {
        showToast('Select a satellite first', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/track', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name })
        });
        
        const data = await response.json();
        if (response.ok) {
            showToast('Tracking started: ' + name, 'success');
            updateTrackingUI(true);
            fetchNextPass();
        } else {
            showToast(data.error || 'Failed to start tracking', 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function stopTracking() {
    try {
        await fetch('/api/track', { method: 'DELETE' });
        showToast('Tracking stopped', 'info');
        updateTrackingUI(false);
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

function updateTrackingUI(tracking) {
    document.getElementById('track-btn').style.display = tracking ? 'none' : 'inline-flex';
    document.getElementById('stop-track-btn').style.display = tracking ? 'inline-flex' : 'none';
}

async function fetchNextPass() {
    try {
        const response = await fetch('/api/nextpass');
        if (response.ok) {
            const data = await response.json();
            displayNextPass(data);
        }
    } catch (e) {
        console.error('Failed to fetch next pass:', e);
    }
}

function displayNextPass(pass) {
    const container = document.getElementById('next-pass');
    const info = document.getElementById('pass-info');
    
    const aos = new Date(pass.aosTime * 1000);
    const los = new Date(pass.losTime * 1000);
    const duration = Math.round(pass.duration / 60);
    
    info.innerHTML = `
        <p><strong>AOS:</strong> ${aos.toLocaleTimeString()} @ ${pass.aosAz.toFixed(0)}°</p>
        <p><strong>Max El:</strong> ${pass.maxEl.toFixed(1)}° @ ${pass.maxElAz.toFixed(0)}°</p>
        <p><strong>LOS:</strong> ${los.toLocaleTimeString()} @ ${pass.losAz.toFixed(0)}°</p>
        <p><strong>Duration:</strong> ${duration} min</p>
    `;
    
    container.style.display = 'block';
}

async function loadTLE() {
    const name = document.getElementById('tle-name').value.trim();
    const line1 = document.getElementById('tle-line1').value.trim();
    const line2 = document.getElementById('tle-line2').value.trim();
    
    if (!name || !line1 || !line2) {
        showToast('Fill in all TLE fields', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/tle', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name, line1, line2 })
        });
        
        const data = await response.json();
        if (response.ok) {
            showToast('TLE loaded: ' + name, 'success');
            loadSatellites();
            // Clear inputs
            document.getElementById('tle-name').value = '';
            document.getElementById('tle-line1').value = '';
            document.getElementById('tle-line2').value = '';
        } else {
            showToast(data.error || 'Failed to load TLE', 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function fetchTLE() {
    const name = document.getElementById('tle-name').value.trim();
    
    if (!name) {
        showToast('Enter satellite name', 'error');
        return;
    }
    
    showToast('Fetching TLE...', 'info');
    
    try {
        const response = await fetch('/api/tle/fetch', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name })
        });
        
        const data = await response.json();
        if (response.ok) {
            document.getElementById('tle-name').value = data.name;
            document.getElementById('tle-line1').value = data.line1;
            document.getElementById('tle-line2').value = data.line2;
            showToast('TLE fetched: ' + data.name, 'success');
            loadSatellites();
        } else {
            showToast(data.error || 'Failed to fetch TLE', 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

// =============================================================================
// GPS & Location
// =============================================================================
async function fetchGPSStatus() {
    try {
        const response = await fetch('/api/gps');
        const data = await response.json();
        
        document.getElementById('gps-status').textContent = data.valid ? '3D Fix ✓' : 'No Fix';
        document.getElementById('gps-status').className = 'gps-value ' + (data.valid ? 'fix' : 'nofix');
        document.getElementById('gps-status-icon').textContent = data.valid ? '✓' : '⏳';
        document.getElementById('gps-lat').textContent = data.latitude?.toFixed(6) ?? '---';
        document.getElementById('gps-lon').textContent = data.longitude?.toFixed(6) ?? '---';
        document.getElementById('gps-alt').textContent = data.altitude ? `${data.altitude.toFixed(1)} m` : '---';
        document.getElementById('gps-sats').textContent = data.satellites ?? '---';
        
        if (data.time) {
            const date = new Date(data.time * 1000);
            document.getElementById('gps-time').textContent = date.toUTCString();
        }
        
        // Pre-fill manual location
        if (data.latitude) {
            document.getElementById('manual-lat').value = data.latitude.toFixed(6);
            document.getElementById('manual-lon').value = data.longitude.toFixed(6);
            document.getElementById('manual-alt').value = data.altitude?.toFixed(1) ?? 0;
        }
    } catch (e) {
        console.error('Failed to fetch GPS status:', e);
    }
}

async function setManualLocation() {
    const lat = parseFloat(document.getElementById('manual-lat').value);
    const lon = parseFloat(document.getElementById('manual-lon').value);
    const alt = parseFloat(document.getElementById('manual-alt').value) || 0;
    
    if (isNaN(lat) || isNaN(lon)) {
        showToast('Invalid coordinates', 'error');
        return;
    }
    
    try {
        const response = await fetch('/api/gps/manual', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ latitude: lat, longitude: lon, altitude: alt })
        });
        
        if (response.ok) {
            showToast('Manual location set', 'success');
            fetchGPSStatus();
        } else {
            const data = await response.json();
            showToast(data.error || 'Failed to set location', 'error');
        }
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

// =============================================================================
// Calibration
// =============================================================================
async function setManualCalibration() {
    const az = parseFloat(document.getElementById('cal-az').value) || 0;
    const el = parseFloat(document.getElementById('cal-el').value) || 0;
    
    try {
        const response = await fetch('/api/calibrate', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ method: 'manual', azOffset: az, elOffset: el })
        });
        
        const data = await response.json();
        showToast(data.message || 'Calibration applied', response.ok ? 'success' : 'error');
        fetchConfig();
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function fetchConfig() {
    try {
        const response = await fetch('/api/config');
        const data = await response.json();
        
        document.getElementById('az-offset').textContent = `${data.azOffset?.toFixed(2) ?? 0}°`;
        document.getElementById('el-offset').textContent = `${data.elOffset?.toFixed(2) ?? 0}°`;
    } catch (e) {
        console.error('Failed to fetch config:', e);
    }
}

// =============================================================================
// WiFi
// =============================================================================
async function fetchWiFiStatus() {
    try {
        const response = await fetch('/api/wifi/status');
        const data = await response.json();
        
        const modes = ['Disconnected', 'Connecting', 'Connected', 'AP Mode'];
        document.getElementById('wifi-status').textContent = modes[data.mode] || 'Unknown';
        document.getElementById('wifi-ssid').textContent = data.ssid || '---';
        document.getElementById('wifi-ip').textContent = data.ip || '---';
        document.getElementById('wifi-rssi').textContent = data.rssi ? `${data.rssi} dBm` : '---';
    } catch (e) {
        console.error('Failed to fetch WiFi status:', e);
    }
}

async function scanNetworks() {
    showToast('Scanning networks...', 'info');
    
    try {
        const response = await fetch('/api/wifi/networks');
        const data = await response.json();
        
        const select = document.getElementById('network-select');
        select.innerHTML = '<option value="">Select Network...</option>';
        
        if (data.networks) {
            data.networks.forEach(net => {
                const option = document.createElement('option');
                option.value = net.ssid;
                option.textContent = `${net.ssid} (${net.rssi} dBm)${net.open ? ' [Open]' : ''}`;
                select.appendChild(option);
            });
        }
        
        showToast(`Found ${data.count} networks`, 'success');
    } catch (e) {
        showToast('Scan failed', 'error');
    }
}

async function connectWiFi() {
    const ssid = document.getElementById('network-select').value;
    const password = document.getElementById('wifi-password').value;
    
    if (!ssid) {
        showToast('Select a network', 'error');
        return;
    }
    
    showToast('Connecting...', 'info');
    
    try {
        const response = await fetch('/api/wifi/connect', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ ssid, password })
        });
        
        const data = await response.json();
        showToast(data.message || 'Connecting...', 'success');
        
        // Wait and check status
        setTimeout(fetchWiFiStatus, 5000);
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

// =============================================================================
// System
// =============================================================================
async function fetchSystemInfo() {
    try {
        const response = await fetch('/api/system/info');
        const data = await response.json();
        
        document.getElementById('sys-version').textContent = data.version || '---';
        document.getElementById('sys-heap').textContent = data.freeHeap ? 
            `${Math.round(data.freeHeap / 1024)} KB` : '---';
        
        // Format uptime
        if (data.uptime) {
            const hours = Math.floor(data.uptime / 3600);
            const minutes = Math.floor((data.uptime % 3600) / 60);
            document.getElementById('sys-uptime').textContent = `${hours}h ${minutes}m`;
        }
    } catch (e) {
        console.error('Failed to fetch system info:', e);
    }
}

async function rebootDevice() {
    if (!confirm('Reboot the tracker?')) return;
    
    try {
        await fetch('/api/system/reboot', { method: 'POST' });
        showToast('Rebooting...', 'info');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

async function factoryReset() {
    if (!confirm('⚠️ This will clear all settings!\n\nAre you sure?')) return;
    if (!confirm('Really clear all settings?')) return;
    
    try {
        await fetch('/api/system/reset', { method: 'POST' });
        showToast('Factory reset complete', 'info');
    } catch (e) {
        showToast('Connection error', 'error');
    }
}

// =============================================================================
// Health Monitoring
// =============================================================================
async function fetchHealth() {
    try {
        const response = await fetch('/api/health');
        const data = await response.json();
        
        // Update health indicators
        updateHealthIndicator('health-gps', data.gpsValid);
        updateHealthIndicator('health-motors', data.azMotorOk && data.elMotorOk);
        updateHealthIndicator('health-wifi', data.wifiConnected);
        updateHealthIndicator('health-sd', data.sdMounted);
        
        // Memory stats
        document.getElementById('health-heap').textContent = 
            `${Math.round(data.freeHeap / 1024)} / ${Math.round(data.minFreeHeap / 1024)} KB`;
        document.getElementById('health-psram').textContent = 
            data.freePsram ? `${Math.round(data.freePsram / 1024)} KB` : 'N/A';
        
        // WiFi RSSI
        document.getElementById('health-rssi').textContent = 
            data.wifiRSSI ? `${data.wifiRSSI} dBm` : '---';
        
        // Uptime
        const hours = Math.floor(data.uptime / 3600);
        const minutes = Math.floor((data.uptime % 3600) / 60);
        document.getElementById('health-uptime').textContent = `${hours}h ${minutes}m`;
        
    } catch (e) {
        console.error('Failed to fetch health:', e);
    }
}

function updateHealthIndicator(id, ok) {
    const el = document.getElementById(id);
    if (el) {
        el.className = `health-indicator ${ok ? 'healthy' : 'unhealthy'}`;
    }
}

// =============================================================================
// Pass Prediction
// =============================================================================
async function fetchPasses() {
    try {
        const response = await fetch('/api/passes');
        if (!response.ok) return;
        
        const data = await response.json();
        displayPasses(data.passes || []);
    } catch (e) {
        console.error('Failed to fetch passes:', e);
    }
}

function displayPasses(passes) {
    const container = document.getElementById('pass-list');
    
    if (!passes || passes.length === 0) {
        container.innerHTML = '<p class="no-data">No upcoming passes</p>';
        return;
    }
    
    container.innerHTML = passes.map((pass, index) => {
        const aos = new Date(pass.aosTime * 1000);
        const duration = Math.round((pass.losTime - pass.aosTime) / 60);
        return `
            <div class="pass-item ${index === 0 ? 'next-pass' : ''}">
                <div class="pass-time">
                    <strong>${aos.toLocaleDateString()} ${aos.toLocaleTimeString([], {hour: '2-digit', minute:'2-digit'})}</strong>
                </div>
                <div class="pass-details">
                    <span>Max: ${pass.maxEl.toFixed(0)}°</span>
                    <span>${duration} min</span>
                </div>
            </div>
        `;
    }).join('');
}

// =============================================================================
// Polar Plot
// =============================================================================
let polarCanvas = null;
let polarCtx = null;

function initPolarPlot() {
    polarCanvas = document.getElementById('polar-plot');
    if (!polarCanvas) return;
    
    polarCtx = polarCanvas.getContext('2d');
    drawPolarGrid();
}

function drawPolarGrid() {
    if (!polarCtx) return;
    
    const width = polarCanvas.width;
    const height = polarCanvas.height;
    const centerX = width / 2;
    const centerY = height / 2;
    const radius = Math.min(width, height) / 2 - 25;
    
    // Clear canvas
    polarCtx.fillStyle = '#0d1218';
    polarCtx.fillRect(0, 0, width, height);
    
    // Draw elevation circles
    polarCtx.strokeStyle = '#1e2a3a';
    polarCtx.lineWidth = 1;
    
    [0, 30, 60, 90].forEach(el => {
        const r = radius * (90 - el) / 90;
        polarCtx.beginPath();
        polarCtx.arc(centerX, centerY, r, 0, 2 * Math.PI);
        polarCtx.stroke();
        
        // Label
        if (el < 90) {
            polarCtx.fillStyle = '#5a6a7e';
            polarCtx.font = '10px sans-serif';
            polarCtx.fillText(`${el}°`, centerX + 3, centerY - r + 12);
        }
    });
    
    // Draw azimuth lines
    for (let az = 0; az < 360; az += 30) {
        const rad = (az - 90) * Math.PI / 180;
        polarCtx.beginPath();
        polarCtx.moveTo(centerX, centerY);
        polarCtx.lineTo(centerX + radius * Math.cos(rad), centerY + radius * Math.sin(rad));
        polarCtx.stroke();
    }
    
    // Cardinal labels
    polarCtx.fillStyle = '#8b9cb3';
    polarCtx.font = 'bold 12px sans-serif';
    polarCtx.textAlign = 'center';
    polarCtx.textBaseline = 'middle';
    polarCtx.fillText('N', centerX, 12);
    polarCtx.fillText('E', width - 12, centerY);
    polarCtx.fillText('S', centerX, height - 12);
    polarCtx.fillText('W', 12, centerY);
}

function updatePolarPlot() {
    if (!polarCtx) return;
    
    drawPolarGrid();
    
    // Draw satellite if tracking
    if (status.satVisible && status.mode === 2) {
        const width = polarCanvas.width;
        const height = polarCanvas.height;
        const centerX = width / 2;
        const centerY = height / 2;
        const radius = Math.min(width, height) / 2 - 25;
        
        const satEl = status.satEl || 0;
        const satAz = status.satAz || 0;
        const r = radius * (90 - satEl) / 90;
        const rad = (satAz - 90) * Math.PI / 180;
        const x = centerX + r * Math.cos(rad);
        const y = centerY + r * Math.sin(rad);
        
        polarCtx.beginPath();
        polarCtx.arc(x, y, 6, 0, 2 * Math.PI);
        polarCtx.fillStyle = '#10b981';
        polarCtx.fill();
        polarCtx.strokeStyle = '#ffffff';
        polarCtx.lineWidth = 2;
        polarCtx.stroke();
    }
}

// =============================================================================
// Toast Notifications
// =============================================================================
function showToast(message, type = 'info') {
    const container = document.getElementById('toast-container');
    
    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.textContent = message;
    
    container.appendChild(toast);
    
    // Animate in
    setTimeout(() => toast.classList.add('show'), 10);
    
    // Remove after delay
    setTimeout(() => {
        toast.classList.remove('show');
        setTimeout(() => toast.remove(), 300);
    }, 4000);
}

// =============================================================================
// Initialization
// =============================================================================
document.addEventListener('DOMContentLoaded', () => {
    initCompass();
    initPolarPlot();
    connectWebSocket();
    fetchConfig();
    
    // Periodic updates
    setInterval(fetchGPSStatus, 10000);
    setInterval(fetchWiFiStatus, 30000);
    setInterval(fetchSystemInfo, 30000);
    setInterval(fetchHealth, 5000);
    setInterval(updatePolarPlot, 1000);
    
    // Initial fetches
    setTimeout(() => {
        fetchPasses();
        fetchHealth();
    }, 2000);
});

// Handle page visibility changes
document.addEventListener('visibilitychange', () => {
    if (document.visibilityState === 'visible') {
        if (!ws || ws.readyState !== WebSocket.OPEN) {
            connectWebSocket();
        }
    }
});
