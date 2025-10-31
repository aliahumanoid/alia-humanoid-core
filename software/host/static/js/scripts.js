// Global variables
let realTimeChart;
let kneeChart;
let ankleDof0Chart, ankleDof1Chart; // Charts for ankle
let hipDof0Chart, hipDof1Chart;     // Charts for hip
let mappingChart;
let mappingCharts = {}; // Object to manage multiple charts for DOFs
let movementChartsMultiDof = {}; // Multi-DOF movement charts
let outputChart;
let measurementData = {
    timestamps: [],
    jointData: {}
};
let isMeasuring = false;
let isMeasuringOutput = false;
let measureOutputData = {
    time: [],
    extensor_output: [],
    flexor_output: []
};
let intervalId;
let realTimeUpdateInterval;
let automaticMappingData = null; // Stores automatic mapping data
let availableSerialPorts = [];
let jointPortMapping = {};
let serialPortAssignmentPending = false;

// Variables for encoder test
let encoderTestInterval = null;
let encoderTestActive = false;
let currentEncoderJointType = null;
let encoderTestData = {
    timestamps: [],
    dofData: {} // Organized as dofData[dof] = {timestamps: [], values: []}
};

// UI configuration for Set Zero and Recalc Offset buttons for each joint/DOF
const JOINT_DOF_UI_CONFIG = {
    KNEE: [
        {
            dof: '0',
            containerId: 'kneeDof0Buttons',
            pretensionLabel: 'Pretension DOF 0',
            releaseLabel: 'Release DOF 0',
            setZeroLabel: 'Set Zero DOF 0',
            setZeroSubtitle: 'Flexion-Extension',
            setZeroClasses: 'bg-yellow-500 hover:bg-yellow-600',
            recalcLabel: 'Recalc Offset DOF 0'
        }
    ],
    ANKLE: [
        {
            dof: '0',
            containerId: 'ankleDof0Buttons',
            pretensionLabel: 'Pretension DOF 0',
            releaseLabel: 'Release DOF 0',
            setZeroLabel: 'Set Zero DOF 0',
            setZeroSubtitle: 'Plantar-Dorsal',
            setZeroClasses: 'bg-yellow-500 hover:bg-yellow-600',
            recalcLabel: 'Recalc Offset DOF 0'
        },
        {
            dof: '1',
            containerId: 'ankleDof1Buttons',
            pretensionLabel: 'Pretension DOF 1',
            releaseLabel: 'Release DOF 1',
            setZeroLabel: 'Set Zero DOF 1',
            setZeroSubtitle: 'Inversion-Eversion',
            setZeroClasses: 'bg-orange-500 hover:bg-orange-600',
            recalcLabel: 'Recalc Offset DOF 1'
        }
    ],
    HIP: [
        {
            dof: '0',
            containerId: 'hipDof0Buttons',
            pretensionLabel: 'Pretension DOF 0',
            releaseLabel: 'Release DOF 0',
            setZeroLabel: 'Set Zero DOF 0',
            setZeroSubtitle: 'Flexion-Extension',
            setZeroClasses: 'bg-yellow-500 hover:bg-yellow-600',
            recalcLabel: 'Recalc Offset DOF 0'
        },
        {
            dof: '1',
            containerId: 'hipDof1Buttons',
            pretensionLabel: 'Pretension DOF 1',
            releaseLabel: 'Release DOF 1',
            setZeroLabel: 'Set Zero DOF 1',
            setZeroSubtitle: 'Abduction-Adduction',
            setZeroClasses: 'bg-orange-500 hover:bg-orange-600',
            recalcLabel: 'Recalc Offset DOF 1'
        }
    ]
};

// Physical joint limits retrieved from backend
let jointPhysicalLimits = {};

// Variables for intelligent status message scrolling
let userScrolledUp = false; // Flag to track if user scrolled up
let autoScrollEnabled = true; // Flag to enable/disable auto-scroll

// Default values (same values in config.py)
const DEFAULT_PARAMS = {
    // Parameters are now managed internally by Pico
};

    // Main function executed when DOM is ready
$(document).ready(function() {
    // Socket.IO initialization
    const socket = io.connect('http://' + document.domain + ':' + location.port + '/movement');

    // Retrieve physical limits from backend
    fetchJointPhysicalLimits().done(function() {
        // Update DOF tabs for initially selected joint after limits are loaded
        const initialJoint = $("#jointSelect").val();
        updateDofTabsAvailability(initialJoint);
        
        // Render DOF-specific buttons after limits are loaded (so zero_angle_offset is available)
        renderDofControlButtons();
    });
    fetchSerialPortConfiguration();
    
    // Load PID values for the initially selected joint (KNEE_LEFT by default)
    setTimeout(function() {
        const initialJoint = $("#jointSelect").val();
        sendCommand('select-joint', { joint: initialJoint });
        console.log('Initial PID values requested for joint:', initialJoint);
    }, 500); // Small delay to ensure backend is ready

    socket.on('connect', function() {
        console.log('Websocket connected!');
    });

    // Listener for multi-DOF movement data
    socket.on('movement_data_multi_dof', function(data) {
        console.log('Movement data multi-DOF received');
        renderMovementChartMultiDof(data);
    });

    socket.on('pid_data', function(data) {
        console.log('PID data received: ', data);
        
        // Handle both old and new format
        if ('kp' in data && 'ki' in data && 'kd' in data) {
            // Old format
            $('#kpInput').val(data.kp);
            $('#kiInput').val(data.ki);
            $('#kdInput').val(data.kd);
        } else if ('joint' in data && 'dof' in data && 'motor_type' in data && 'values' in data) {
            // New format
            const dof = data.dof;
            const motorType = data.motor_type;
            const values = data.values;
            
            // Get ID prefix based on motor type (1=agonist, 2=antagonist)
            const prefix = motorType === 1 ? 'agonist' : 'antagonist';
            
            // Update corresponding PID input fields
            $(`#${prefix}PidDof${dof}Kp`).val(values.kp);
            $(`#${prefix}PidDof${dof}Ki`).val(values.ki);
            $(`#${prefix}PidDof${dof}Kd`).val(values.kd);
            $(`#${prefix}PidDof${dof}Tau`).val(values.tau);
            
            // Activate DOF tab if not already active
            showPidTab(dof);
        }
    });

    socket.on('pid_outer_data', function(data) {
        console.log('PID outer loop data received: ', data);

        if (!data || !data.values) {
            return;
        }

        const selectedJoint = $("#jointSelect").val();
        if (data.joint && data.joint !== selectedJoint) {
            return;
        }

        const dof = data.dof;
        const values = data.values;

        const formatValue = (val, digits) => Number.isFinite(val) ? val.toFixed(digits) : val;

        $(`#outerPidDof${dof}Kp`).val(formatValue(values.kp, 4));
        $(`#outerPidDof${dof}Ki`).val(formatValue(values.ki, 4));
        $(`#outerPidDof${dof}Kd`).val(formatValue(values.kd, 4));
        $(`#outerPidDof${dof}Stiffness`).val(formatValue(values.stiffness, 3));
        $(`#outerPidDof${dof}Cascade`).val(formatValue(values.cascade, 3));

        const stiffnessText = Number.isFinite(values.stiffness) ? values.stiffness.toFixed(2) : values.stiffness;
        const cascadeText = Number.isFinite(values.cascade) ? (values.cascade * 100).toFixed(1) : values.cascade;
        const kpText = Number.isFinite(values.kp) ? values.kp.toFixed(3) : values.kp;
        const kiText = Number.isFinite(values.ki) ? values.ki.toFixed(3) : values.ki;
        const kdText = Number.isFinite(values.kd) ? values.kd.toFixed(3) : values.kd;

        appendStatusMessage(`External PID received for DOF ${dof}: Kp=${kpText}, Ki=${kiText}, Kd=${kdText}, Stiff=${stiffnessText}°, Cascade=${cascadeText}%`);
    });

    // Listener for mapping data from MAPPING_DATA protocol
    socket.on('automatic_mapping_data', function(data) {
        console.log('Mapping data received via SocketIO:', data);
        
        // Store received data
        automaticMappingData = data.data;
        
        // If data contains joint name, store it
        if (data.joint_name) {
            window.lastActiveJointFromSocket = data.joint_name;
            console.log(`Joint received in mapping data: ${data.joint_name}`);
            
            // Update combo box if necessary
            const jointSelect = $("#jointSelect");
            if (jointSelect.length && jointSelect.val() !== data.joint_name) {
                jointSelect.val(data.joint_name);
                appendStatusMessage(`🔄 Joint updated from mapping data: ${data.joint_name}`);
            }
        }
        
        // Use actual DOF count from received data
        const effectiveDofCount = data.dof_count || data.data.actual_dof_count || 1;
        
        // Automatically update mapping chart with new data
        renderMappingChart({
            total_points: data.total_points,
            dof_count: effectiveDofCount,
            data: data.data,
            joint_name: data.joint_name  // Pass joint name
        });
        
        // Regenerate smart buttons with new mapping data
        setTimeout(() => {
            generateSmartQuickButtons();
        }, 200);
        
        // Show status message
        const jointInfo = data.joint_name ? ` for ${data.joint_name}` : '';
        appendStatusMessage(`Mapping data received${jointInfo}: ${data.total_points} points, ${effectiveDofCount} effective DOF`);
    });

    // Listener for real-time data from sockets
    socket.on('joint_measure', function(data) {
        if (!isMeasuring) return;
        
        const timestamp = data.timestamp;
        const joint = data.joint;
        const dof = data.dof;
        const measurements = data.data;
        
        // Add timestamp if new
        if (!measurementData.timestamps.includes(timestamp)) {
            measurementData.timestamps.push(timestamp);
            // Limit number of displayed points
            if (measurementData.timestamps.length > 100) {
                measurementData.timestamps.shift();
            }
        }
        
        // Save data for each measurement field
        for (const [motor, value] of Object.entries(measurements)) {
            const dataKey = `${joint}_${dof}_${motor}`;
            if (!measurementData.jointData[dataKey]) {
                measurementData.jointData[dataKey] = [];
            }
            
            measurementData.jointData[dataKey].push({
                x: timestamp,
                y: value
            });
            
            // Limit number of points for each series
            if (measurementData.jointData[dataKey].length > 100) {
                measurementData.jointData[dataKey].shift();
            }
        }
    });
    
    socket.on('update_active_joint', function(data) {
        // Update UI combo boxes when active joint changes
        if (data && data.joint) {
            console.log(`Active joint update: ${data.joint} (source: ${data.source})`);
            
            // Store active joint received via socket
            window.lastActiveJointFromSocket = data.joint;
            
            // Update joint combo box
            const jointSelect = $("#jointSelect");
            if (jointSelect.length && jointSelect.val() !== data.joint) {
                jointSelect.val(data.joint);
                appendStatusMessage(`🔄 Active joint updated automatically: ${data.joint}`);
            }
            
            // If source is a mapping request, update charts
            if (data.source === 'mapping_request' || data.source === 'automatic_mapping') {
                setTimeout(() => {
                    fetchMappingChartData();
                }, 500); // Brief pause to allow combo box update
            }
        }
    });

    // Initialize charts
    initializeCharts();
    
    // Handle joint change
    $("#jointSelect").change(function() {
        const joint = $(this).val();
        updateSerialPortSelectUI(joint);
        
        // Update DOF tab availability based on joint configuration
        updateDofTabsAvailability(joint);
        
        // Regenerate DOF control buttons with new joint's zero_angle_offset
        renderDofControlButtons();
        
        // Stop encoder test if active (to avoid conflicts)
        if (encoderTestActive) {
            stopEncoderTest();
            appendStatusMessage("🔄 Encoder test stopped due to joint change");
        }
        
        // Reset encoder UI for all joints
        resetAllEncoderUI();
        
        updateJointPanels();
        
        // Automatically load mapping data for new joint
        setTimeout(() => {
            fetchMappingChartData();
        }, 100);
        
        // Regenerate smart buttons for new joint
        setTimeout(() => {
            generateSmartQuickButtons();
        }, 150); // Slightly longer delay to allow mapping data loading
        
        // Send command to select joint and load PIDs
        sendCommand('select-joint', { joint: joint });
    });

    // DOF selection removed - now per-DOF controls handle specific DOF operations

    $("#serialPortSelect").change(function() {
        const joint = $("#jointSelect").val();
        const port = $(this).val() || null;
        assignSerialPortToJoint(joint, port);
    });

    $("#refreshSerialPorts").on('click', function() {
        fetchSerialPortConfiguration({ showStatus: true });
    });

    // Start status polling
    startPolling();

    // Wait a moment before requesting PID data
    setTimeout(fetchPID, 500); 
    
    // Initialize smart buttons on load
    setTimeout(generateSmartQuickButtons, 250);
    
    // DOF-specific buttons are now initialized in fetchJointPhysicalLimits().done() callback
    
    // Configure listener for intelligent status message scrolling
    const statusMessages = $("#statusMessages");
    let scrollTimeout;
    
    statusMessages.on('scroll', function() {
        const element = this;
        
        // Debounce scroll control for performance
        clearTimeout(scrollTimeout);
        scrollTimeout = setTimeout(function() {
            const isAtBottom = isScrolledToBottom(element);
            
            if (isAtBottom) {
                // User is at bottom - re-enable auto-scroll
                userScrolledUp = false;
                autoScrollEnabled = true;
            } else {
                // User scrolled up - disable auto-scroll
                userScrolledUp = true;
                autoScrollEnabled = false;
            }
            
            // Update visual indicator
            updateScrollIndicator();
        }, 150); // 150ms debounce
    });
});

// --- Globally defined functions --- 

// updateDofOptions() removed - DOF selector no longer exists in UI

/**
 * Updates the visibility of joint-specific panels based on selected joint
 * Shows/hides KNEE, ANKLE, or HIP panels accordingly
 */
function updateJointPanels() {
    const joint = $("#jointSelect").val();
    const jointType = joint.split('_')[0]; // Extract KNEE, ANKLE or HIP from full name
    
    $("#kneePanel, #anklePanel, #hipPanel").hide();
    
    if (jointType === "KNEE") {
        $("#kneePanel").show();
    } else if (jointType === "ANKLE") {
        $("#anklePanel").show();
        // Ankle temporal charts are initialized automatically
    } else if (jointType === "HIP") {
        $("#hipPanel").show();
        // Hip temporal charts are initialized automatically
    }
}

function initializeCharts() {
    // Real Time Chart
    const rtCtx = document.getElementById('realTimeChart').getContext('2d');
    realTimeChart = new Chart(rtCtx, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: { scales: { x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (s)' } }, y: { beginAtZero: false, title: { display: true, text: 'Value' } } }, animation: false, plugins: { legend: { position: 'top' } } }
    });
    
    // Knee Chart
    const kneeCtx = document.getElementById('kneeChart').getContext('2d');
    kneeChart = new Chart(kneeCtx, {
        type: 'line',
        data: { labels: [], datasets: [{ label: 'Extensor', data: [], borderColor: 'rgba(255, 99, 132, 1)', borderWidth: 1, fill: false }, { label: 'Flexor', data: [], borderColor: 'rgba(54, 162, 235, 1)', borderWidth: 1, fill: false }] },
        options: { scales: { y: { beginAtZero: false }, x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (s)' } } } }
    });
    
    // Configure temporal charts for ankle and hip with minimalist style
    const timeChartConfig = (title, color) => ({
        type: 'line',
        data: { 
            datasets: [{ 
                label: title,
                data: [], 
                borderColor: color, 
                backgroundColor: color.replace('1)', '0.1)'),
                borderWidth: 2, 
                fill: false,
                pointRadius: 0,
                pointHoverRadius: 3
            }] 
        },
        options: { 
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: {
                intersect: false,
                mode: 'index'
            },
            scales: { 
                x: { 
                    type: 'linear',
                    position: 'bottom',
                    min: 0,
                    max: 100,
                    title: { 
                        display: true, 
                        text: 'Time (s)',
                        font: { size: 10 }
                    },
                    ticks: { 
                        stepSize: 20,
                        font: { size: 9 }
                    },
                    grid: {
                        color: 'rgba(0, 0, 0, 0.1)'
                    }
                }, 
                y: { 
                    beginAtZero: false,
                    title: { 
                        display: true, 
                        text: 'Gradi',
                        font: { size: 10 }
                    },
                    ticks: {
                        font: { size: 9 }
                    },
                    grid: {
                        color: 'rgba(0, 0, 0, 0.1)'
                    }
                } 
            },
            plugins: {
                legend: { 
                    display: false  // Minimize legend
                },
                title: {
                    display: false
                }
            }
        }
    });
    
    // Ankle Charts DOF 0 and DOF 1
    const ankleDof0Ctx = document.getElementById('ankleDof0Chart').getContext('2d');
    ankleDof0Chart = new Chart(ankleDof0Ctx, timeChartConfig('DOF 0', 'rgba(54, 162, 235, 1)'));
    
    const ankleDof1Ctx = document.getElementById('ankleDof1Chart').getContext('2d');
    ankleDof1Chart = new Chart(ankleDof1Ctx, timeChartConfig('DOF 1', 'rgba(255, 99, 132, 1)'));
    
    // Hip Charts DOF 0 and DOF 1
    const hipDof0Ctx = document.getElementById('hipDof0Chart').getContext('2d');
    hipDof0Chart = new Chart(hipDof0Ctx, timeChartConfig('DOF 0', 'rgba(75, 192, 192, 1)'));
    
    const hipDof1Ctx = document.getElementById('hipDof1Chart').getContext('2d');
    hipDof1Chart = new Chart(hipDof1Ctx, timeChartConfig('DOF 1', 'rgba(153, 102, 255, 1)'));

    // Initialize multi-DOF mapping charts system
    initializeMappingChartsSystem();

    // Initialize multi-DOF movement charts system
    initializeMovementChartsSystem();

    // Output Chart
    const outCtx = document.getElementById('outputChart').getContext('2d');
    outputChart = new Chart(outCtx, {
        type: 'line',
        data: { labels: [], datasets: [{ label: 'Extensor Output', data: [], borderColor: 'rgba(255, 99, 132, 1)', borderWidth: 1, fill: false }, { label: 'Flexor Output', data: [], borderColor: 'rgba(54, 162, 235, 1)', borderWidth: 1, fill: false }] },
        options: { animation: { duration: 0 }, scales: { y: { beginAtZero: true }, x: { type: 'linear', position: 'bottom' } }, responsive: false, maintainAspectRatio: false }
    });
}

// === MULTI-DOF TEMPORAL CHARTS MANAGEMENT ===

// Variables to manage timing and chart reset
let chartStartTimes = {}; // Track start time for each chart
const CHART_RESET_TIME = 100; // Reset after 100 seconds

/**
 * Adds a data point to the specified temporal chart with automatic reset management
 * @param {Chart} chart - Chart.js chart instance
 * @param {string} chartId - Unique chart ID for time tracking
 * @param {number} value - Angular value in degrees
 */
function addTimePointToChart(chart, chartId, value) {
    if (!chart || !chart.data || !chart.data.datasets || chart.data.datasets.length === 0) return;
    
    const currentTime = Date.now();
    
    // Initialize start time if it doesn't exist
    if (!chartStartTimes[chartId]) {
        chartStartTimes[chartId] = currentTime;
    }
    
    // Calculate relative time in seconds
    const relativeTimeSeconds = (currentTime - chartStartTimes[chartId]) / 1000;
    
    // If we're beyond 100 seconds, reset chart
    if (relativeTimeSeconds > CHART_RESET_TIME) {
        // Reset all datasets
        chart.data.datasets.forEach(dataset => {
            dataset.data = [];
        });
        chartStartTimes[chartId] = currentTime;
        const newRelativeTime = 0;
        
        // Add new point to appropriate dataset (last if encoder, otherwise first)
        const targetDataset = getTargetDatasetForValue(chart, chartId);
        if (targetDataset) {
            targetDataset.data.push({
                x: newRelativeTime,
                y: value
            });
        }
        
        appendStatusMessage(`🔄 Chart ${chartId} reset after ${CHART_RESET_TIME}s`);
    } else {
        // Add point normally to appropriate dataset
        const targetDataset = getTargetDatasetForValue(chart, chartId);
        if (targetDataset) {
            targetDataset.data.push({
                x: relativeTimeSeconds,
                y: value
            });
            
            // Remove points that are too old (over 105 seconds for safety)
            // This prevents excessive accumulation in case of reset issues
            targetDataset.data = targetDataset.data.filter(point => 
                (currentTime - chartStartTimes[chartId]) / 1000 - point.x <= 105
            );
        }
    }
    
    // Update X axis to always show 0 to 100
    chart.options.scales.x.min = 0;
    chart.options.scales.x.max = CHART_RESET_TIME;
    
    // Update chart without animations
    chart.update('none');
}

/**
 * Determines target dataset for value (handles charts with multiple datasets like knee)
 * @param {Chart} chart - Chart.js chart instance
 * @param {string} chartId - Chart ID
 * @returns {object|null} Target dataset or null if not found
 */
function getTargetDatasetForValue(chart, chartId) {
    if (!chart.data.datasets || chart.data.datasets.length === 0) return null;
    
    // For encoder charts, find encoder dataset (the one with "Encoder" in name)
    if (chartId.includes('knee') || chartId.includes('ankle') || chartId.includes('hip')) {
        const encoderDataset = chart.data.datasets.find(dataset => 
            dataset.label && dataset.label.includes('Encoder')
        );
        if (encoderDataset) {
            return encoderDataset;
        }
    }
    
    // Fallback: use first available dataset
    return chart.data.datasets[0];
}

/**
 * Manual reset of a temporal chart
 * @param {Chart} chart - Chart.js chart instance
 * @param {string} chartId - Unique chart ID
 */
function resetTemporalChart(chart, chartId) {
    if (!chart || !chart.data || !chart.data.datasets || chart.data.datasets.length === 0) return;
    
    // Reset di tutti i dataset
    chart.data.datasets.forEach(dataset => {
        dataset.data = [];
    });
    chartStartTimes[chartId] = Date.now();
    chart.update('none');
    
    appendStatusMessage(`🔄 Chart ${chartId} manually reset`);
}

function initializeMappingChartsSystem() {
    // Initialize multi-DOF mapping charts system
    // Container is prepared, charts are created dynamically when needed
    const container = document.getElementById('mappingChartsContainer');
    if (container) {
        container.innerHTML = '<p class="text-center text-gray-500 py-8">Select a joint to view mapping charts</p>';
    }
}

function createMappingChartsForDof(dof, containerElement) {
    /**
     * Creates two mapping charts for a specific DOF (measured and interpolated data)
     * 
     * @param {number} dof - DOF number
     * @param {HTMLElement} containerElement - Container element for charts
     * @returns {object} - Object with instances of both charts {measured: Chart, interpolated: Chart}
     */
    
    // Create container for measured data chart
    const measuredDiv = document.createElement('div');
    measuredDiv.className = 'mb-4';
    measuredDiv.innerHTML = `<h4 class="text-md font-semibold mb-2 text-gray-700">Measured Data</h4>`;
    
    // Create wrapper container with fixed height for canvas
    const measuredWrapper = document.createElement('div');
    measuredWrapper.style.height = '250px';
    measuredWrapper.style.position = 'relative';
    
    const measuredCanvas = document.createElement('canvas');
    measuredCanvas.id = `mappingChartDof${dof}Measured`;
    measuredWrapper.appendChild(measuredCanvas);
    measuredDiv.appendChild(measuredWrapper);
    containerElement.appendChild(measuredDiv);
    
    // Create container for interpolated data chart
    const interpolatedDiv = document.createElement('div');
    interpolatedDiv.innerHTML = `<h4 class="text-md font-semibold mb-2 text-gray-700">Linear Interpolation</h4>`;
    
    // Create wrapper container with fixed height for canvas
    const interpolatedWrapper = document.createElement('div');
    interpolatedWrapper.style.height = '250px';
    interpolatedWrapper.style.position = 'relative';
    
    const interpolatedCanvas = document.createElement('canvas');
    interpolatedCanvas.id = `mappingChartDof${dof}Interpolated`;
    interpolatedWrapper.appendChild(interpolatedCanvas);
    interpolatedDiv.appendChild(interpolatedWrapper);
    containerElement.appendChild(interpolatedDiv);
    
    // Create chart for measured data
    const measuredCtx = measuredCanvas.getContext('2d');
    const measuredChart = new Chart(measuredCtx, {
        type: 'line',
        data: { 
            labels: [], 
            datasets: [
                { 
                    label: 'Agonista (Misurato)', 
                    data: [], 
                    borderColor: 'rgba(255, 99, 132, 1)', 
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 2, 
                    fill: false,
                    pointRadius: 4,
                    pointHoverRadius: 6,
                    showLine: false // Only points for original data
                }, 
                { 
                    label: 'Antagonista (Misurato)', 
                    data: [], 
                    borderColor: 'rgba(54, 162, 235, 1)', 
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 2, 
                    fill: false,
                    pointRadius: 4,
                    pointHoverRadius: 6,
                    showLine: false // Only points for original data
                }
            ] 
        },
        options: { 
            responsive: true,
            maintainAspectRatio: false,
            resizeDelay: 0,
            animation: false,
            scales: { 
                y: { 
                    beginAtZero: false,
                    title: {
                        display: true,
                        text: 'Angolo Motore (gradi)'
                    }
                }, 
                x: { 
                    type: 'linear', 
                    position: 'bottom', 
                    title: {
                        display: true,
                        text: `Angolo Giunto DOF ${dof} (gradi)`
                    },
                    ticks: { 
                        stepSize: 5, 
                        beginAtZero: false 
                    } 
                } 
            },
            plugins: {
                title: {
                    display: true,
                    text: `DOF ${dof} - Experimental Data`,
                    font: { size: 12 }
                },
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            interaction: {
                intersect: false,
                mode: 'index'
            }
        }
    });
    
    // Create chart for interpolated data
    const interpolatedCtx = interpolatedCanvas.getContext('2d');
    const interpolatedChart = new Chart(interpolatedCtx, {
        type: 'line',
        data: { 
            labels: [], 
            datasets: [
                { 
                    label: 'Agonista (Interpolato)', 
                    data: [], 
                    borderColor: 'rgba(255, 99, 132, 0.9)', 
                    backgroundColor: 'rgba(255, 99, 132, 0.1)',
                    borderWidth: 3, 
                    fill: false,
                    pointRadius: 2,
                    pointHoverRadius: 4,
                    borderDash: [5, 5] // Dashed line for interpolation
                },
                { 
                    label: 'Antagonista (Interpolato)', 
                    data: [], 
                    borderColor: 'rgba(54, 162, 235, 0.9)', 
                    backgroundColor: 'rgba(54, 162, 235, 0.1)',
                    borderWidth: 3, 
                    fill: false,
                    pointRadius: 2,
                    pointHoverRadius: 4,
                    borderDash: [5, 5] // Dashed line for interpolation
                },
                { 
                    label: 'Agonista (Estrapolato)', 
                    data: [], 
                    borderColor: 'rgba(255, 99, 132, 0.5)', 
                    backgroundColor: 'rgba(255, 99, 132, 0.05)',
                    borderWidth: 2, 
                    fill: false,
                    pointRadius: 1,
                    pointHoverRadius: 3,
                    borderDash: [10, 10] // More dashed line for extrapolation
                },
                { 
                    label: 'Antagonista (Estrapolato)', 
                    data: [], 
                    borderColor: 'rgba(54, 162, 235, 0.5)', 
                    backgroundColor: 'rgba(54, 162, 235, 0.05)',
                    borderWidth: 2, 
                    fill: false,
                    pointRadius: 1,
                    pointHoverRadius: 3,
                    borderDash: [10, 10] // More dashed line for extrapolation
                }
            ] 
        },
        options: { 
            responsive: true,
            maintainAspectRatio: false,
            resizeDelay: 0,
            animation: false,
            scales: { 
                y: { 
                    beginAtZero: false,
                    title: {
                        display: true,
                        text: 'Angolo Motore (gradi)'
                    }
                }, 
                x: { 
                    type: 'linear', 
                    position: 'bottom', 
                    title: {
                        display: true,
                        text: `Angolo Giunto DOF ${dof} (gradi)`
                    },
                    ticks: { 
                        stepSize: 5, 
                        beginAtZero: false 
                    } 
                } 
            },
            plugins: {
                title: {
                    display: true,
                    text: `DOF ${dof} - Interpolation + Extrapolation (+10%)`,
                    font: { size: 12 }
                },
                legend: {
                    display: true,
                    position: 'top'
                }
            },
            interaction: {
                intersect: false,
                mode: 'index'
            }
        }
    });
    
    return {
        measured: measuredChart,
        interpolated: interpolatedChart
    };
}

function calculateLinearRegression(xData, yData) {
    /**
     * Calculates linear regression using least squares method
     * 
     * @param {number[]} xData - Array of X values
     * @param {number[]} yData - Array of Y values
     * @returns {object} - {slope: m, intercept: b, r2: coefficient of determination}
     */
    
    if (xData.length !== yData.length || xData.length < 2) {
        return null;
    }
    
    const n = xData.length;
    const sumX = xData.reduce((sum, x) => sum + x, 0);
    const sumY = yData.reduce((sum, y) => sum + y, 0);
    const sumXY = xData.reduce((sum, x, i) => sum + x * yData[i], 0);
    const sumXX = xData.reduce((sum, x) => sum + x * x, 0);
    const sumYY = yData.reduce((sum, y) => sum + y * y, 0);
    
    // Calcolo pendenza (slope) e intercetta usando minimi quadrati
    const slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    const intercept = (sumY - slope * sumX) / n;
    
    // Calcolo coefficiente di determinazione R²
    const yMean = sumY / n;
    const ssTotal = yData.reduce((sum, y) => sum + Math.pow(y - yMean, 2), 0);
    const ssResidual = yData.reduce((sum, y, i) => {
        const predicted = slope * xData[i] + intercept;
        return sum + Math.pow(y - predicted, 2);
    }, 0);
    const r2 = 1 - (ssResidual / ssTotal);
    
    const slopeFormatted = slope.toFixed(4);
    let interceptValue = intercept;
    if (Math.abs(interceptValue) < 1e-8) {
        interceptValue = 0;
    }
    const interceptFormatted = Math.abs(interceptValue).toFixed(4);
    const sign = interceptValue >= 0 ? '+' : '-';
    const equation = interceptValue === 0
        ? `y = ${slopeFormatted}x`
        : `y = ${slopeFormatted}x ${sign} ${interceptFormatted}`;

    return {
        slope: slope,
        intercept: intercept,
        r2: r2,
        equation: equation,
        r2Text: `R² = ${r2.toFixed(4)}`
    };
}

function generateInterpolatedPoints(regression, minX, maxX, stepSize = 1) {
    /**
     * Generates interpolated points using regression equation
     * 
     * @param {object} regression - Linear regression result
     * @param {number} minX - Minimum X-axis value
     * @param {number} maxX - Maximum X-axis value
     * @param {number} stepSize - Interval between points (default 1 degree)
     * @returns {object[]} - Array of points {x, y}
     */
    
    if (!regression) return [];
    
    const points = [];
    for (let x = minX; x <= maxX; x += stepSize) {
        const y = regression.slope * x + regression.intercept;
        points.push({ x: x, y: y });
    }
    
    return points;
}

function generateInterpolatedAndExtrapolatedPoints(regression, minX, maxX, stepSize = 1, extrapolationPercent = 10) {
    /**
     * Generates interpolated and extrapolated points using regression equation
     * Includes extrapolation beyond measured data limits
     * 
     * @param {object} regression - Linear regression result
     * @param {number} minX - Minimum X-axis value from measured data
     * @param {number} maxX - Maximum X-axis value from measured data
     * @param {number} stepSize - Interval between points (default 1 degree)
     * @param {number} extrapolationPercent - Extrapolation percentage beyond limits (default 10%)
     * @returns {object} - {interpolated: Array, extrapolated: Array, full: Array, extendedRange: {min, max}}
     */
    
    if (!regression) return {
        interpolated: [],
        extrapolated: [],
        full: [],
        extendedRange: { min: minX, max: maxX }
    };
    
    // Calculate extrapolation range
    const range = maxX - minX;
    const extension = range * (extrapolationPercent / 100);
    
    const extendedMinX = minX - extension;
    const extendedMaxX = maxX + extension;
    
    // Round to nearest integer degrees for clean values
    const roundedExtendedMinX = Math.floor(extendedMinX);
    const roundedExtendedMaxX = Math.ceil(extendedMaxX);
    
    // Generate interpolated points (original range)
    const interpolatedPoints = [];
    for (let x = minX; x <= maxX; x += stepSize) {
        const y = regression.slope * x + regression.intercept;
        interpolatedPoints.push({ 
            x: x, 
            y: y, 
            type: 'interpolated'
        });
    }
    
    // Generate extrapolated points (extensions only)
    const extrapolatedPoints = [];
    
    // Downward extrapolation (minimum)
    for (let x = roundedExtendedMinX; x < minX; x += stepSize) {
        const y = regression.slope * x + regression.intercept;
        extrapolatedPoints.push({ 
            x: x, 
            y: y, 
            type: 'extrapolated_low'
        });
    }
    
    // Upward extrapolation (maximum)
    for (let x = maxX + stepSize; x <= roundedExtendedMaxX; x += stepSize) {
        const y = regression.slope * x + regression.intercept;
        extrapolatedPoints.push({ 
            x: x, 
            y: y, 
            type: 'extrapolated_high'
        });
    }
    
    // Combine all points and sort by x
    const allPoints = [...interpolatedPoints, ...extrapolatedPoints];
    allPoints.sort((a, b) => a.x - b.x);
    
    return {
        interpolated: interpolatedPoints,
        extrapolated: extrapolatedPoints,
        full: allPoints,
        extendedRange: {
            min: roundedExtendedMinX,
            max: roundedExtendedMaxX,
            originalMin: minX,
            originalMax: maxX,
            extension: extension
        }
    };
}

function enrichMappingDataWithInterpolation(mappingData) {
    /**
     * Enriches mapping data with interpolated and extrapolated points
     * 
     * @param {object} mappingData - Original mapping data
     * @returns {object} - Enriched data with interpolation and extrapolation
     */
    
    if (!mappingData || !mappingData.present_dofs) {
        return mappingData;
    }
    
    const enrichedData = JSON.parse(JSON.stringify(mappingData)); // Deep copy
    
    // Process each DOF
    mappingData.present_dofs.forEach(dof => {
        const dofData = mappingData[`dof_${dof}`];
        if (!dofData || !dofData.joint_angles || !dofData.agonist_angles || !dofData.antagonist_angles) {
            return;
        }
        
        // Filter valid points
        const validPoints = [];
        for (let i = 0; i < dofData.joint_angles.length; i++) {
            if (dofData.joint_angles[i] !== null && dofData.agonist_angles[i] !== null && 
                dofData.antagonist_angles[i] !== null) {
                validPoints.push({
                    joint: dofData.joint_angles[i],
                    agonist: dofData.agonist_angles[i],
                    antagonist: dofData.antagonist_angles[i]
                });
            }
        }
        
        if (validPoints.length < 2) return;
        
        // Sort points
        validPoints.sort((a, b) => a.joint - b.joint);
        
        const jointAngles = validPoints.map(p => p.joint);
        const agonistAngles = validPoints.map(p => p.agonist);
        const antagonistAngles = validPoints.map(p => p.antagonist);
        
        const minJointAngle = Math.min(...jointAngles);
        const maxJointAngle = Math.max(...jointAngles);
        
        // Calculate linear regressions
        const agonistRegression = calculateLinearRegression(jointAngles, agonistAngles);
        const antagonistRegression = calculateLinearRegression(jointAngles, antagonistAngles);
        
        if (agonistRegression && antagonistRegression) {
            // Calculate intelligent step size based on original data (as in backend)
            const originalSteps = [];
            for (let i = 0; i < jointAngles.length - 1; i++) {
                const step = Math.abs(jointAngles[i + 1] - jointAngles[i]);
                if (step > 0) {
                    originalSteps.push(step);
                }
            }
            
            let smartStepSize = 1; // Default
            if (originalSteps.length > 0) {
                // Use median of original steps, rounded to nearest degree
                originalSteps.sort((a, b) => a - b);
                const medianStep = originalSteps[Math.floor(originalSteps.length / 2)];
                smartStepSize = Math.max(1, Math.round(medianStep));
                console.log(`DOF ${dof}: Smart step size JS = ${smartStepSize}° (original median: ${medianStep.toFixed(2)}°)`);
            }
            
            // Generate interpolated and extrapolated points with smart step
            const agonistExtended = generateInterpolatedAndExtrapolatedPoints(
                agonistRegression, minJointAngle, maxJointAngle, smartStepSize, 10
            );
            const antagonistExtended = generateInterpolatedAndExtrapolatedPoints(
                antagonistRegression, minJointAngle, maxJointAngle, smartStepSize, 10
            );
            
            // Add enriched data
            enrichedData[`dof_${dof}`].interpolation = {
                agonist: {
                    regression: agonistRegression,
                    interpolated_points: agonistExtended.interpolated,
                    extrapolated_points: agonistExtended.extrapolated,
                    full_points: agonistExtended.full,
                    extended_range: agonistExtended.extendedRange
                },
                antagonist: {
                    regression: antagonistRegression,
                    interpolated_points: antagonistExtended.interpolated,
                    extrapolated_points: antagonistExtended.extrapolated,
                    full_points: antagonistExtended.full,
                    extended_range: antagonistExtended.extendedRange
                },
                metadata: {
                    original_data_points: validPoints.length,
                    interpolated_points_count: agonistExtended.interpolated.length,
                    extrapolated_points_count: agonistExtended.extrapolated.length,
                    total_points_count: agonistExtended.full.length,
                    extrapolation_percent: 10,
                    step_size: smartStepSize,
                    original_median_step: originalSteps.length > 0 ? originalSteps[Math.floor(originalSteps.length / 2)] : "N/A"
                }
            };
        }
    });
    
    return enrichedData;
}

function exportInterpolationEquations(jointName) {
    /**
     * Exports interpolation equations for all joint DOFs
     * 
     * @param {string} jointName - Nome del giunto
     */
    
    if (!automaticMappingData) {
        appendStatusMessage("No mapping data available for export");
        return;
    }
    
    // Enrich data with interpolation and extrapolation
    const enrichedData = enrichMappingDataWithInterpolation(automaticMappingData);
    
    const exportData = {
        joint_name: jointName,
        timestamp: new Date().toISOString(),
        original_data: automaticMappingData,
        enriched_data: enrichedData,
        equations: {},
        interpolation_summary: {
            total_dofs: 0,
            total_original_points: 0,
            total_interpolated_points: 0,
            total_extrapolated_points: 0,
            extrapolation_percent: 10
        }
    };
    
    // Process each available DOF
    const availableDofs = automaticMappingData.present_dofs || [];
    availableDofs.forEach(dof => {
        const dofData = enrichedData[`dof_${dof}`];
        if (dofData && dofData.interpolation) {
            
            exportData.equations[`dof_${dof}`] = {
                agonist: {
                    regression: dofData.interpolation.agonist.regression,
                    extended_range: dofData.interpolation.agonist.extended_range,
                    interpolated_points: dofData.interpolation.agonist.interpolated_points,
                    extrapolated_points: dofData.interpolation.agonist.extrapolated_points,
                    full_points: dofData.interpolation.agonist.full_points
                },
                antagonist: {
                    regression: dofData.interpolation.antagonist.regression,
                    extended_range: dofData.interpolation.antagonist.extended_range,
                    interpolated_points: dofData.interpolation.antagonist.interpolated_points,
                    extrapolated_points: dofData.interpolation.antagonist.extrapolated_points,
                    full_points: dofData.interpolation.antagonist.full_points
                },
                metadata: dofData.interpolation.metadata,
                original_measured_data: {
                    joint_angles: dofData.joint_angles,
                    agonist_angles: dofData.agonist_angles,
                    antagonist_angles: dofData.antagonist_angles
                }
            };
            
            // Update summary
            exportData.interpolation_summary.total_dofs++;
            exportData.interpolation_summary.total_original_points += dofData.interpolation.metadata.original_data_points;
            exportData.interpolation_summary.total_interpolated_points += dofData.interpolation.metadata.interpolated_points_count;
            exportData.interpolation_summary.total_extrapolated_points += dofData.interpolation.metadata.extrapolated_points_count;
        }
    });
    
    // Create and download JSON file
    const jsonStr = JSON.stringify(exportData, null, 2);
    const dataBlob = new Blob([jsonStr], { type: 'application/json' });
    const url = URL.createObjectURL(dataBlob);
    
    const link = document.createElement('a');
    link.href = url;
    link.download = `${jointName.toLowerCase()}_interpolation_equations_${new Date().toISOString().slice(0, 10)}.json`;
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    URL.revokeObjectURL(url);
    
    const summary = exportData.interpolation_summary;
    appendStatusMessage(`Exported complete file for ${jointName}: ${summary.total_dofs} DOF, ${summary.total_original_points} original points, ${summary.total_interpolated_points} interpolated, ${summary.total_extrapolated_points} extrapolated (+10%)`);
}

/**
 * Requests PID parameters from the controller
 */
function fetchPID() { sendCommand('get-pid'); }

/**
 * Sends a command to the backend for the currently selected joint
 * @param {string} command - Command name
 * @param {object} additionalData - Additional data to send with the command
 */
function sendCommand(command, additionalData = {}) {
    const joint = $("#jointSelect").val();
    const assignedPort = jointPortMapping[joint];

    if (!assignedPort) {
        appendStatusMessage(`⚠️ Associate a serial port to ${joint} before sending the ${command.toUpperCase()} command.`);
        return;
    }

    // Use DOF from additionalData if provided, otherwise default to 'ALL'
    const dof = additionalData.dof !== undefined ? additionalData.dof : 'ALL';
    
    // Remove dof from additionalData to avoid duplication, then spread the rest
    const { dof: _, ...otherData } = additionalData;
    const data = { cmd: command, joint: joint, dof: dof, ...otherData };
    
    $.ajax({
        url: "/command", method: "POST", data: JSON.stringify(data), contentType: "application/json; charset=utf-8", dataType: "json",
        success: function(response) { appendStatusMessage(response.message); },
        error: function(xhr, status, error) { appendStatusMessage(`Error sending ${command}: ${error}`); }
    });
}

/**
 * Sends pretension command for a specific DOF
 * @param {string|number} dofValue - DOF index ('0', '1', '2') or 'ALL'
 */
function sendPretension(dofValue) {
    // DOF must be explicitly provided (e.g., '0', '1', 'ALL')
    if (dofValue === undefined || dofValue === null) {
        appendStatusMessage('⚠️ Error: DOF not specified for pretension command');
        return;
    }
    sendCommand('pretension', { dof: dofValue });
    const joint = $("#jointSelect").val();
    appendStatusMessage(`Pretension sent for ${joint} DOF ${dofValue}`);
}

/**
 * Sends release command for a specific DOF
 * @param {string|number} dofValue - DOF index ('0', '1', '2') or 'ALL'
 */
function sendRelease(dofValue) {
    // DOF must be explicitly provided (e.g., '0', '1', 'ALL')
    if (dofValue === undefined || dofValue === null) {
        appendStatusMessage('⚠️ Error: DOF not specified for release command');
        return;
    }
    sendCommand('release', { dof: dofValue });
    const joint = $("#jointSelect").val();
    appendStatusMessage(`Release sent for ${joint} DOF ${dofValue}`);
}

function startMeasureOutput() {
    if (isMeasuring) return;
    sendCommand('start-measure-output');
    isMeasuring = true;
    measurementData = { timestamps: [], jointData: {} };
    startRealTimeUpdates();
}

function stopMeasureOutput() {
    if (!isMeasuring) return;
    sendCommand('stop-measure-output');
    isMeasuring = false;
    stopRealTimeUpdates();
}

function startRealTimeUpdates() {
    stopRealTimeUpdates();
    realTimeUpdateInterval = setInterval(updateRealTimeChart, 100);
}

function stopRealTimeUpdates() { if (realTimeUpdateInterval) { clearInterval(realTimeUpdateInterval); } }

function updateRealTimeChart() {
    if (!isMeasuring || !realTimeChart) return;
    const datasets = [];
    for (const [dataKey, values] of Object.entries(measurementData.jointData)) {
        const parts = dataKey.split('_');
        if (parts.length >= 3) {
            const joint = parts[0]; const dof = parts[1]; const motor = parts.slice(2).join('_');
            let color = 'rgba(153, 102, 255, 1)'; // Default
            if (motor.includes('extensor') || motor.includes('plantar') || motor.includes('flex_ext')) { color = 'rgba(255, 99, 132, 1)'; }
            else if (motor.includes('flexor') || motor.includes('dorsal') || motor.includes('abd_add')) { color = 'rgba(54, 162, 235, 1)'; }
            else if (motor.includes('inversion')) { color = 'rgba(255, 206, 86, 1)'; }
            else if (motor.includes('eversion')) { color = 'rgba(75, 192, 192, 1)'; }
            datasets.push({ label: `${joint} ${dof} ${motor}`, data: values, borderColor: color, backgroundColor: color.replace('1)', '0.1)'), borderWidth: 1, fill: false, pointRadius: 0 });
        }
    }
    realTimeChart.data.datasets = datasets;
    realTimeChart.update('none'); // Update without animations for fluidity
}

function saveMeasurementData() {
    let csvContent = "data:text/csv;charset=utf-8,";
    let headers = ["Timestamp"];
    for (const dataKey of Object.keys(measurementData.jointData)) { headers.push(dataKey); }
    csvContent += headers.join(",") + "\r\n";
    for (let i = 0; i < measurementData.timestamps.length; i++) {
        let row = [measurementData.timestamps[i]];
        for (const dataKey of Object.keys(measurementData.jointData)) {
            const point = measurementData.jointData[dataKey].find(p => p.x === measurementData.timestamps[i]);
            row.push(point ? point.y : "");
        }
        csvContent += row.join(",") + "\r\n";
    }
    const encodedUri = encodeURI(csvContent);
    const link = document.createElement("a");
    link.setAttribute("href", encodedUri);
    link.setAttribute("download", `joint_measurements_${new Date().toISOString().replace(/:/g, '-')}.csv`);
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
}

function startPolling() {
    stopPolling(); // Assicura che non ci siano polling multipli
    intervalId = setInterval(function() {
        $.ajax({
            url: "/status_message", method: "GET", dataType: "json",
            success: function(response) { if (response.status === "success") { appendStatusMessage(response.message); } },
            error: function() { /* Silently ignore polling errors */ }
        });
    }, 100); // Riduco intervallo di polling a 100ms
}

function stopPolling() { if (intervalId) { clearInterval(intervalId); } }

function appendStatusMessage(message) {
    const statusMessages = $("#statusMessages");
    
    // Avoid consecutive duplicate messages
    if (statusMessages.children().length === 0 || statusMessages.children().last().text() !== message) {
        // Check if user is already at bottom before adding new message
        const wasAtBottom = isScrolledToBottom(statusMessages[0]);
        
        // Add new message
        statusMessages.append(`<li>${message}</li>`);
        
        // Auto-scroll only if user was already at bottom (or close)
        // and if auto-scroll is enabled
        if ((wasAtBottom || !userScrolledUp) && autoScrollEnabled) {
            statusMessages.scrollTop(statusMessages[0].scrollHeight);
        }
    }
    
    // Limit number of messages - increased to keep more history
    while (statusMessages.children().length > 200) { // Aumentato da 50 a 200 messaggi
        statusMessages.children().first().remove();
    }
}

/**
 * Checks if element is scrolled to bottom (with 30px tolerance)
 * @param {HTMLElement} element - Element to check
 * @returns {boolean} True if element is scrolled to bottom
 */
function isScrolledToBottom(element) {
    const threshold = 30; // Tolerance in pixels
    return element.scrollHeight - element.clientHeight <= element.scrollTop + threshold;
}

/**
 * Forces scroll to most recent messages and re-enables auto-scroll
 */
function scrollToLatestMessage() {
    const statusMessages = $("#statusMessages");
    statusMessages.scrollTop(statusMessages[0].scrollHeight);
    userScrolledUp = false;
    autoScrollEnabled = true;
    
    // Visually update button if present
    updateScrollIndicator();
    appendStatusMessage("📜 Scrolled to latest messages - auto-scroll riattivato");
}

/**
 * Aggiorna l'indicatore visuale dello stato dello scroll
 */
function updateScrollIndicator() {
    const scrollButton = $("#scrollToBottomBtn");
    const statusContainer = $("#statusMessages").parent(); // Now it's the div with class "relative"
    
    if (userScrolledUp && !autoScrollEnabled) {
        // Show button if user scrolled up
        if (scrollButton.length === 0) {
            statusContainer.append(`
                <button id="scrollToBottomBtn" 
                        onclick="scrollToLatestMessage()" 
                        class="absolute bottom-2 right-2 bg-blue-500 hover:bg-blue-600 text-white text-xs py-1 px-2 rounded shadow-lg z-10 transition-all">
                    <i class="fas fa-arrow-down mr-1"></i>Nuovi messaggi
                </button>
            `);
        }
    } else {
        // Hide button if not needed
        scrollButton.remove();
    }
}

// --- Serial port management ---

function fetchSerialPortConfiguration(options = {}) {
    const { showStatus = false } = options;
    return $.ajax({
        url: '/serial_ports',
        method: 'GET',
        dataType: 'json'
    }).done(response => {
        availableSerialPorts = response.ports || [];
        jointPortMapping = response.mappings || {};
        updateSerialPortSelectUI($("#jointSelect").val());
        if (showStatus) {
            appendStatusMessage(`🔌 Porte seriali aggiornate (${availableSerialPorts.length})`);
        }
    }).fail((xhr, status, error) => {
        appendStatusMessage(`⚠️ Impossibile recuperare le porte seriali: ${error}`);
    });
}

function getJointUsingPort(port) {
    for (const [joint, mappedPort] of Object.entries(jointPortMapping)) {
        if (mappedPort === port) {
            return joint;
        }
    }
    return null;
}

function updateSerialPortSelectUI(joint) {
    const select = $("#serialPortSelect");
    if (!select.length) {
        return;
    }

    const currentSelection = select.val();
    select.empty();
    select.append('<option value="">Select port</option>');

    availableSerialPorts.forEach(port => {
        const ownerJoint = getJointUsingPort(port);
        let label = port;
        if (ownerJoint && ownerJoint !== joint) {
            label = `${port} (in uso da ${ownerJoint})`;
        }
        const option = $('<option></option>').val(port).text(label);
        select.append(option);
    });

    const assignedPort = jointPortMapping[joint] || '';
    if (assignedPort && !availableSerialPorts.includes(assignedPort)) {
        select.append($('<option></option>').val(assignedPort).text(`${assignedPort} (non rilevata)`));
    }

    if (assignedPort) {
        select.val(assignedPort);
    } else if (currentSelection) {
        select.val('');
    }

    updateSerialPortHint(joint);
}

function updateSerialPortHint(joint) {
    const hint = $("#serialPortHint");
    if (!hint.length) {
        return;
    }
    const port = jointPortMapping[joint];
    if (port) {
        const ownerJoint = getJointUsingPort(port);
        if (ownerJoint && ownerJoint !== joint) {
            hint.text(`Port ${port} currently assigned to ${ownerJoint}`);
        } else {
            hint.text(`Associated port: ${port}`);
        }
    } else {
        hint.text('No port associated with this joint');
    }
}

function assignSerialPortToJoint(joint, port) {
    if (serialPortAssignmentPending) {
        return;
    }
    serialPortAssignmentPending = true;

    $.ajax({
        url: '/serial_mapping',
        method: 'POST',
        contentType: 'application/json; charset=utf-8',
        dataType: 'json',
        data: JSON.stringify({ joint, port }),
        success: function(response) {
            jointPortMapping = response.mappings || jointPortMapping;
            updateSerialPortSelectUI(joint);

            const result = response.result || {};
            if (result.port) {
                if (result.reassigned_joint && result.reassigned_joint !== joint) {
                    appendStatusMessage(`♻️ Port ${result.port} moved from ${result.reassigned_joint} to ${joint}`);
                } else {
                    appendStatusMessage(`🔗 ${joint} associated with port ${result.port}`);
                }
                
                // Load PID values from the newly connected device
                setTimeout(function() {
                    sendCommand('select-joint', { joint: joint });
                    console.log('PID values requested after serial port change for:', joint);
                }, 300); // Short delay to allow connection to stabilize
            } else {
                appendStatusMessage(`🔌 Serial association removed for ${joint}`);
            }
        },
        error: function(xhr, status, error) {
            appendStatusMessage(`⚠️ Error during serial assignment: ${error}`);
            updateSerialPortSelectUI(joint);
        },
        complete: function() {
            serialPortAssignmentPending = false;
        }
    });
}

// --- Legacy chart functions (Mapping, Movement, Output) ---

function fetchMappingChartData() {
    const selectedJoint = $("#jointSelect").val();
    
    // First try to load saved data for selected joint
    $.ajax({
        url: `/get_saved_mapping_data/${selectedJoint}`,
        method: "GET",
        dataType: "json",
        success: function(response) {
            console.log("Saved mapping data received for", selectedJoint, ":", response); // Debug
            if (response.has_data) {
                // Use saved mapping data
                automaticMappingData = response.data;
                
                // Determine actual number of DOF and points
                const dofCount = response.data.actual_dof_count || response.data.dof_count || 1;
                const totalPoints = response.data.total_points || 0;
                
                renderMappingChart({
                    total_points: totalPoints,
                    dof_count: dofCount,
                    data: response.data
                });
                
                const timestamp = response.file_timestamp ? new Date(response.file_timestamp).toLocaleString() : 'N/A';
                appendStatusMessage(`Loaded saved data for ${selectedJoint}: ${totalPoints} points, ${dofCount} DOF (${timestamp})`);
                updateMappingDataInfo(`${selectedJoint}: ${totalPoints} points, ${dofCount} DOF - Saved: ${timestamp}`);
            } else {
                // If no saved data, try with in-memory data
                loadMemoryMappingData(selectedJoint);
            }
        },
        error: function(xhr, status, error) {
            console.log("Error loading saved data for", selectedJoint, ", trying with in-memory data"); // Debug
            // If error with saved data, try with in-memory data
            loadMemoryMappingData(selectedJoint);
        }
    });
}

function loadMemoryMappingData(jointName) {
    if (!jointPortMapping[jointName]) {
        appendStatusMessage(`ℹ️ No serial port associated with ${jointName}, cannot retrieve current data.`);
        updateMappingDataInfo(`No active data for ${jointName}`);
        return;
    }

    $.ajax({
        url: "/get_mapping_data",
        method: "GET",
        data: { joint: jointName },
        dataType: "json",
        success: function(response) {
            console.log("In-memory mapping data received:", response); // Debug
            if (response.has_data) {
                // Use mapping data in memory
                automaticMappingData = response.data;
                
                // Determine actual number of DOF and points
                const dofCount = response.data.actual_dof_count || response.data.dof_count || 1;
                const totalPoints = response.data.total_points || 0;
                
                renderMappingChart({
                    total_points: totalPoints,
                    dof_count: dofCount,
                    data: response.data
                });
                appendStatusMessage(`Loaded in-memory data: ${totalPoints} points, ${dofCount} DOF (current)`);
                updateMappingDataInfo(`Current data: ${totalPoints} points, ${dofCount} DOF`);
            } else {
                appendStatusMessage(`No mapping data available for ${jointName}`);
                updateMappingDataInfo(`No data available for ${jointName}`);
            }
        },
        error: function(xhr, status, error) {
            console.error("Error loading mapping data:", error); // Debug
            appendStatusMessage(`Error loading mapping data for ${jointName}`);
            updateMappingDataInfo('Error loading');
        }
    });
}

function renderMappingChart(mappingData) {
    const selectedJoint = $("#jointSelect").val();
    
    // Store data for future use
    automaticMappingData = mappingData.data;
    
    // Enrich data with interpolation and extrapolation
    const enrichedData = enrichMappingDataWithInterpolation(mappingData.data);
    
    // Determine correct joint from received data or combo box
    // Priority: 1) direct parameter, 2) internal data, 3) socket, 4) combo box
    let jointForSaving = selectedJoint;
    let source = 'combo box';
    
    if (mappingData.joint_name) {
        // Direct parameter from call (highest priority)
        jointForSaving = mappingData.joint_name;
        source = 'call parameter';
    } else if (mappingData.data && mappingData.data.joint_name) {
        // If data contains joint name, use it
        jointForSaving = mappingData.data.joint_name;
        source = 'mapping data';
    } else if (window.lastActiveJointFromSocket) {
        // If we received joint update via socket, use it
        jointForSaving = window.lastActiveJointFromSocket;
        source = 'socket';
    }
    
    // Automatically save enriched data (in background)
    if (enrichedData && enrichedData.present_dofs && enrichedData.present_dofs.length > 0) {
        console.log(`🔄 Saving enriched data for joint: ${jointForSaving} (source: ${source})`);
        saveEnrichedMappingData(jointForSaving, enrichedData);
    }
    
    // Get main container
    const container = document.getElementById('mappingChartsContainer');
    if (!container) {
        appendStatusMessage("Error: mapping charts container not found");
        return;
    }
    
    // Clear existing charts
    clearAllMappingCharts();
    
    // Identify available DOFs in data
    const availableDofs = mappingData.data.present_dofs || [];
    if (!availableDofs || availableDofs.length === 0) {
        // Fallback: search for available DOFs
        for (let i = 0; i < mappingData.dof_count; i++) {
            if (mappingData.data[`dof_${i}`]) {
                availableDofs.push(i);
            }
        }
    }
    
    if (availableDofs.length === 0) {
        container.innerHTML = '<p class="text-center text-red-500 py-8">No DOF available in mapping data</p>';
        appendStatusMessage("No DOF available in mapping data");
        return;
    }
    
    // Always display all available DOFs
    const dofsToShow = availableDofs;
    
    // Create HTML structure for charts
    const gridClass = dofsToShow.length === 1 ? 'grid-cols-1' : 
                     dofsToShow.length === 2 ? 'grid-cols-1 md:grid-cols-2' : 
                     'grid-cols-1 md:grid-cols-2 lg:grid-cols-3';
    
    container.innerHTML = `<div class="grid ${gridClass} gap-6"></div>`;
    const gridContainer = container.querySelector('.grid');
    
    // Create chart for each DOF to display
    dofsToShow.forEach(dof => {
        const dofData = mappingData.data[`dof_${dof}`];
        if (!dofData) {
            appendStatusMessage(`Data not found for DOF ${dof}`);
            return;
        }
        
        // Create container for this DOF
        const dofContainer = document.createElement('div');
        dofContainer.className = 'bg-gray-50 rounded-lg p-4';
        dofContainer.innerHTML = `
            <h3 class="text-lg font-semibold mb-3 text-center">DOF ${dof}</h3>
            <div class="chart-container-dof-${dof}"></div>
        `;
        gridContainer.appendChild(dofContainer);
        
        // Create charts for this DOF (measured and interpolated)
        const chartContainer = dofContainer.querySelector(`.chart-container-dof-${dof}`);
        const charts = createMappingChartsForDof(dof, chartContainer);
        
        // Populate chart with data
        if (dofData.joint_angles && dofData.agonist_angles && dofData.antagonist_angles) {
            // Filter valid points (remove null/undefined values)
            const validPoints = [];
            for (let i = 0; i < dofData.joint_angles.length; i++) {
                if (dofData.joint_angles[i] !== null && dofData.joint_angles[i] !== undefined &&
                    dofData.agonist_angles[i] !== null && dofData.agonist_angles[i] !== undefined &&
                    dofData.antagonist_angles[i] !== null && dofData.antagonist_angles[i] !== undefined) {
                    validPoints.push({
                        joint: dofData.joint_angles[i],
                        agonist: dofData.agonist_angles[i],
                        antagonist: dofData.antagonist_angles[i]
                    });
                }
            }
            
            if (validPoints.length > 0) {
                // Sort points by joint angle for clearer visualization
                validPoints.sort((a, b) => a.joint - b.joint);
                
                const jointAngles = validPoints.map(p => p.joint);
                const agonistAngles = validPoints.map(p => p.agonist);
                const antagonistAngles = validPoints.map(p => p.antagonist);
                
                // Calculate range for interpolation
                const minJointAngle = Math.min(...jointAngles);
                const maxJointAngle = Math.max(...jointAngles);
                
                // Calculate linear regressions
                const agonistRegression = calculateLinearRegression(jointAngles, agonistAngles);
                const antagonistRegression = calculateLinearRegression(jointAngles, antagonistAngles);
                
                // Calculate smart step size for charts (same algorithm used in enrichMappingDataWithInterpolation)
                const originalSteps = [];
                for (let i = 0; i < jointAngles.length - 1; i++) {
                    const step = Math.abs(jointAngles[i + 1] - jointAngles[i]);
                    if (step > 0) {
                        originalSteps.push(step);
                    }
                }
                
                let graphStepSize = 1; // Default
                if (originalSteps.length > 0) {
                    originalSteps.sort((a, b) => a - b);
                    const medianStep = originalSteps[Math.floor(originalSteps.length / 2)];
                    graphStepSize = Math.max(1, Math.round(medianStep));
                    console.log(`DOF ${dof} Chart: Smart step size = ${graphStepSize}° (median: ${medianStep.toFixed(2)}°)`);
                }
                
                // Populate chart with measured data
                charts.measured.data.datasets[0].data = agonistAngles.map((y, i) => ({ x: jointAngles[i], y: y }));
                charts.measured.data.datasets[1].data = antagonistAngles.map((y, i) => ({ x: jointAngles[i], y: y }));
                
                const safeRange = computeJointSafeRange(selectedJoint, dof, jointAngles);
                const safeMin = safeRange ? safeRange.min : minJointAngle;
                const safeMax = safeRange ? safeRange.max : maxJointAngle;

                charts.interpolated.data.datasets[0].data = generateLinearPointsWithinRange(
                    agonistRegression,
                    safeMin,
                    safeMax,
                    graphStepSize
                );
                charts.interpolated.data.datasets[1].data = generateLinearPointsWithinRange(
                    antagonistRegression,
                    safeMin,
                    safeMax,
                    graphStepSize
                );

                // Extrapolation datasets are no longer displayed with simplified approach
                charts.interpolated.data.datasets[2].data = [];
                charts.interpolated.data.datasets[3].data = [];

                charts.measured.update('none'); // Update without animations to avoid resizing
                charts.interpolated.update('none'); // Update without animations to avoid resizing
                
                // Log equations for debug
                if (agonistRegression) {
                    console.log(`DOF ${dof} Agonista: ${agonistRegression.equation}, ${agonistRegression.r2Text}`);
                    appendStatusMessage(`DOF ${dof} Agonista: ${agonistRegression.equation} (${agonistRegression.r2Text})`);
                }
                if (antagonistRegression) {
                    console.log(`DOF ${dof} Antagonista: ${antagonistRegression.equation}, ${antagonistRegression.r2Text}`);
                    appendStatusMessage(`DOF ${dof} Antagonista: ${antagonistRegression.equation} (${antagonistRegression.r2Text})`);
                }
                
                const rangeForMessage = safeRange || { min: minJointAngle, max: maxJointAngle };
                appendStatusMessage(`DOF ${dof}: ${validPoints.length} original points • equation shown in safe range ${rangeForMessage.min.toFixed(2)}° → ${rangeForMessage.max.toFixed(2)}° (original data: ${minJointAngle.toFixed(2)}° → ${maxJointAngle.toFixed(2)}°)`);
            } else {
                appendStatusMessage(`DOF ${dof}: no valid points found`);
            }
        } else {
            appendStatusMessage(`DOF ${dof}: incomplete data (missing angles)`);
        }
        
        // Store charts for future reference
        mappingCharts[`dof_${dof}_measured`] = charts.measured;
        mappingCharts[`dof_${dof}_interpolated`] = charts.interpolated;
    });
    
    // Update mapping data info
    const dofInfo = selectedDof === "ALL" ? 
        `${dofsToShow.length} DOF (${dofsToShow.join(', ')})` : 
        `DOF ${dofsToShow[0]}`;
    
    updateMappingDataInfo(`${selectedJoint}: ${mappingData.total_points} points, ${dofInfo}`);
    
    // Regenerate smart buttons after loading new mapping data
    setTimeout(() => {
        generateSmartQuickButtons();
    }, 100);
    
    appendStatusMessage(`Displayed mapping charts for ${selectedJoint}: ${dofInfo}`);
}



function fetchOutputData() {
    if (!isMeasuringOutput) return;
    const joint = $("#jointSelect").val();
    $.ajax({
        url: "/get_output_data",
        method: "GET",
        data: { joint },
        dataType: "json",
        success: function(response) {
            if (!response || response.status !== "success" || !response.data) {
                console.warn('No output data available for', joint, response);
                setTimeout(fetchOutputData, 500);
                return;
            }

            const data = response.data;
            measureOutputData.time.push(data.time);
            measureOutputData.extensor_output.push(data.extensor_output);
            measureOutputData.flexor_output.push(data.flexor_output);
            if (measureOutputData.time.length > 100) { // Limit data points
                measureOutputData.time.shift();
                measureOutputData.extensor_output.shift();
                measureOutputData.flexor_output.shift();
            }
            renderOutputChart();
            setTimeout(fetchOutputData, 200);
        },
        error: function(xhr, status, error) {
            console.error('Error retrieving output data:', error);
            setTimeout(fetchOutputData, 500);
        }
    });
}

function renderOutputChart() {
    if (!outputChart) return;
    outputChart.data.labels = measureOutputData.time;
    outputChart.data.datasets[0].data = measureOutputData.extensor_output;
    outputChart.data.datasets[1].data = measureOutputData.flexor_output;
    outputChart.update();
}

// Function to update PID for selected DOF and motor type
function updatePidForDofMotor(dof, motorType) {
    const prefix = motorType === 1 ? 'agonist' : 'antagonist';
    const motorName = motorType === 1 ? 'Agonist' : 'Antagonist';
    
    const kp = $(`#${prefix}PidDof${dof}Kp`).val();
    const ki = $(`#${prefix}PidDof${dof}Ki`).val();
    const kd = $(`#${prefix}PidDof${dof}Kd`).val();
    const tau = $(`#${prefix}PidDof${dof}Tau`).val();
    
    sendCommand('set-pid', { 
        dof: dof, 
        motor_type: motorType, 
        kp: kp, 
        ki: ki, 
        kd: kd, 
        tau: tau 
    });
    
    appendStatusMessage(`PID parameters set for DOF ${dof} motor ${motorName}`);
}

// Function to request PIDs for specific DOF and motor type
function requestPidForDofMotor(dof, motorType) {
    sendCommand('get-pid', { dof: dof, motor_type: motorType });
}

function requestOuterPidForDof(dof) {
    sendCommand('get-pid-outer', { dof: dof });
}

// Function to show PID tab for specific DOF
function showPidTab(dofIndex) {
    // Hide all tabs
    $('.pid-tab').hide();
    
    // Show selected tab
    $(`#pidTabDof${dofIndex}`).show();
    
    // Update active class of tab selector
    $('.pid-tab-selector').removeClass('bg-blue-500').addClass('bg-gray-300');
    $(`#pidTabSelector${dofIndex}`).removeClass('bg-gray-300').addClass('bg-blue-500');
}

// Function to update DOF tab availability based on joint configuration
function updateDofTabsAvailability(jointName) {
    const jointLimits = jointPhysicalLimits?.[jointName];
    const dofCount = jointLimits?.dof_count || 1; // Default to 1 DOF if not specified
    
    // Enable/disable tab selectors (DOF 0, 1, 2)
    for (let i = 0; i < 3; i++) {
        const tabSelector = $(`#pidTabSelector${i}`);
        if (i < dofCount) {
            // Enable tab
            tabSelector.prop('disabled', false);
            tabSelector.removeClass('opacity-50 cursor-not-allowed');
            tabSelector.addClass('hover:bg-blue-600 hover:bg-gray-400');
        } else {
            // Disable tab
            tabSelector.prop('disabled', true);
            tabSelector.removeClass('hover:bg-blue-600 hover:bg-gray-400');
            tabSelector.addClass('opacity-50 cursor-not-allowed');
        }
    }
    
    // If current active DOF is disabled, switch to DOF 0
    const activeTabIndex = $('.pid-tab-selector.bg-blue-500').index();
    if (activeTabIndex >= dofCount) {
        showPidTab(0);
    }
    
    console.log(`Updated DOF tabs for ${jointName}: ${dofCount} DOF(s) available`);
}

// Function to load all PIDs for current joint
function loadAllPids() {
    sendCommand('load-pid-all');
    appendStatusMessage("All PID request sent");
}

function updateOuterPidForDof(dof) {
    const kp = parseFloat($(`#outerPidDof${dof}Kp`).val()) || 0;
    const ki = parseFloat($(`#outerPidDof${dof}Ki`).val()) || 0;
    const kd = parseFloat($(`#outerPidDof${dof}Kd`).val()) || 0;
    const stiffness = parseFloat($(`#outerPidDof${dof}Stiffness`).val()) || 0;
    let cascade = parseFloat($(`#outerPidDof${dof}Cascade`).val());
    if (!Number.isFinite(cascade)) cascade = 0;
    cascade = Math.min(Math.max(cascade, 0), 1);

    sendCommand('set-pid-outer', {
        dof: dof,
        kp: kp,
        ki: ki,
        kd: kd,
        stiffness: stiffness,
        cascade: cascade
    });

    appendStatusMessage(`External PID sent for DOF ${dof}: Kp=${kp}, Ki=${ki}, Kd=${kd}, Stiffness=${stiffness}°, Cascade=${(cascade * 100).toFixed(1)}%`);
}

// --- Mapping chart management functions ---

function clearAllMappingCharts() {
    // Destroys all existing mapping charts
    Object.values(mappingCharts).forEach(chart => {
        if (chart && typeof chart.destroy === 'function') {
            chart.destroy();
        }
    });
    
    // Reset charts object
    mappingCharts = {};
    
    // Clear container
    const container = document.getElementById('mappingChartsContainer');
    if (container) {
        container.innerHTML = '<p class="text-center text-gray-500 py-8">Select a joint to view mapping charts</p>';
    }
    
    updateMappingDataInfo('No data loaded');
    appendStatusMessage("Mapping charts cleared");
}

// === MULTI-DOF MOVEMENT CHARTS SYSTEM ===

function initializeMovementChartsSystem() {
    // System is initialized but charts are created dynamically
    // when data arrives, based on present DOFs
    clearAllMovementCharts();
    appendStatusMessage("Multi-DOF movement charts system initialized");
}

function renderMovementChartMultiDof(movementData) {
    if (!movementData || !movementData.joints) {
        console.warn("Invalid multi-DOF movement data");
        return;
    }
    
    const container = document.getElementById('movementChartsMultiDofContainer');
    if (!container) {
        console.error("Multi-DOF movement charts container not found");
        return;
    }
    
    // Get metadata
    const metadata = movementData.metadata || {};
    const jointName = metadata.source_joint || "UNKNOWN";
    const sourceDof = metadata.source_dof || "ALL";
    const totalSamples = metadata.total_samples || 0;
    const arrayType = metadata.array_type || 7;
    
    appendStatusMessage(`📈 Received movement data for ${jointName} DOF ${sourceDof}: ${totalSamples} samples`);
    
    // Clear all previous charts before rendering new ones
    clearAllMovementCharts();
    
    // Clear the container
    container.innerHTML = '';
    
    // Process each joint
    Object.entries(movementData.joints).forEach(([joint, jointData]) => {
        // Process each DOF of joint
        Object.entries(jointData).forEach(([dofKey, dofData]) => {
            // Handle both numeric keys (0, 1, 2) and string keys ("dof_0", "dof_1", etc.)
            let dofNumber;
            if (typeof dofKey === 'number' || /^\d+$/.test(dofKey)) {
                // Direct numeric key
                dofNumber = dofKey.toString();
            } else if (dofKey.startsWith('dof_')) {
                // String key with "dof_" prefix
                dofNumber = dofKey.replace('dof_', '');
            } else {
                console.warn('Skipping unknown DOF key format:', dofKey);
                return;
            }

            // If chart already exists for this DOF, destroy and recreate it (update)
            const chartKey = `${joint}_${dofNumber}`;
            if (movementChartsMultiDof[chartKey]) {
                // Distruggi le eventuali istanze Chart esistenti
                Object.values(movementChartsMultiDof[chartKey]).forEach(chart => {
                    if (chart && typeof chart.destroy === 'function') {
                        chart.destroy();
                    }
                });
                // Rimuovi il container DOM esistente se presente
                const existing = document.getElementById(`movementDofContainer_${joint}_${dofNumber}`);
                if (existing && existing.parentNode) {
                    existing.parentNode.removeChild(existing);
                }
                delete movementChartsMultiDof[chartKey];
            }

            // Create (or recreate) charts for this DOF
            createMovementChartForDof(joint, dofNumber, dofData, container, arrayType);
        });
    });
    
    // Aggiorna il pulsante di refresh
    updateMovementDataRefreshButton(jointName, sourceDof, totalSamples);
}

function createMovementChartForDof(joint, dof, dofData, container, arrayType) {
    // Create container for this DOF
    const dofContainer = document.createElement('div');
    dofContainer.className = 'mb-6 bg-gray-50 p-4 rounded-lg border border-gray-200';
    dofContainer.id = `movementDofContainer_${joint}_${dof}`;
    
    // DOF header
    const motorNames = dofData.metadata?.motor_names || {agonist: 'agonist', antagonist: 'antagonist'};
    const headerHtml = `
        <div class="flex justify-between items-center mb-4">
            <h3 class="text-lg font-semibold text-gray-800">
                ${joint} - DOF ${dof} 
                <span class="text-sm text-gray-600">(${motorNames.agonist}/${motorNames.antagonist})</span>
            </h3>
            <div class="text-sm text-gray-500">
                ${dofData.joint_angles?.length || 0} samples • Array Type ${arrayType}
            </div>
        </div>
    `;
    
    dofContainer.innerHTML = headerHtml;
    
    // Container for stacked charts
    const chartsContainer = document.createElement('div');
    chartsContainer.className = 'space-y-4';
    
    // Chart 1: Joint Angles over Time
    const jointAngleChart = createJointAngleChart(joint, dof, dofData, motorNames);
    chartsContainer.appendChild(jointAngleChart);
    
    // Chart 2: Current vs Next Motor Angles
    const motorAnglesChart = createMotorAnglesChart(joint, dof, dofData, motorNames);
    chartsContainer.appendChild(motorAnglesChart);
    
    // Chart 3: Motor Torques (array_type=7 or 8)
    if ((arrayType === 7 || arrayType === 8) && dofData.motor_torques) {
        const motorTorquesChart = createMotorTorquesChart(joint, dof, dofData, motorNames);
        chartsContainer.appendChild(motorTorquesChart);
    }
    
    dofContainer.appendChild(chartsContainer);
    container.appendChild(dofContainer);
    
    // Store charts for future management
    const chartKey = `${joint}_${dof}`;
    if (!movementChartsMultiDof[chartKey]) {
        movementChartsMultiDof[chartKey] = {};
    }
}

function createJointAngleChart(joint, dof, dofData, motorNames) {
    const chartContainer = document.createElement('div');
    chartContainer.className = 'bg-white p-3 rounded border';
    chartContainer.innerHTML = `
        <h4 class="text-sm font-medium text-gray-700 mb-2">Angolo Giunto nel Tempo</h4>
        <div style="height: 200px;">
            <canvas id="jointAngleChart_${joint}_${dof}"></canvas>
        </div>
    `;
    
    // Configure chart after element is in DOM
    setTimeout(() => {
        const ctx = document.getElementById(`jointAngleChart_${joint}_${dof}`);
        if (!ctx) return;
        
        const jointTargetAngles = [...(dofData.joint_angles || [])];
        const jointActualAngles = [...(dofData.joint_angles_actual || [])];
        const maxSamples = Math.max(jointTargetAngles.length, jointActualAngles.length);
        const timeData = Array.from({ length: maxSamples }, (_, idx) => idx);

        const datasets = [];
        if (jointTargetAngles.length > 0) {
            while (jointTargetAngles.length < maxSamples) {
                jointTargetAngles.push(null);
            }
            datasets.push({
                label: 'Angolo Target (°)',
                data: jointTargetAngles,
                borderColor: 'rgba(75, 192, 192, 1)',
                backgroundColor: 'rgba(75, 192, 192, 0.1)',
                borderWidth: 2,
                fill: true,
                tension: 0.1
            });
        }

        if (jointActualAngles.length > 0) {
            while (jointActualAngles.length < maxSamples) {
                jointActualAngles.push(null);
            }
            datasets.push({
                label: 'Angolo Effettivo (°)',
                data: jointActualAngles,
                borderColor: 'rgba(255, 99, 132, 1)',
                backgroundColor: 'rgba(255, 99, 132, 0.1)',
                borderWidth: 2,
                fill: false,
                tension: 0.1
            });
        }

        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: timeData,
                datasets: datasets
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        title: { display: true, text: 'Campione' }
                    },
                    y: {
                        title: { display: true, text: 'Gradi (°)' }
                    }
                },
                plugins: {
                    legend: { display: datasets.length > 1 }
                }
            }
        });
        
        // Memorizza il grafico
        const chartKey = `${joint}_${dof}`;
        if (!movementChartsMultiDof[chartKey]) {
            movementChartsMultiDof[chartKey] = {};
        }
        movementChartsMultiDof[chartKey].jointAngle = chart;
    }, 100);
    
    return chartContainer;
}

function createMotorAnglesChart(joint, dof, dofData, motorNames) {
    const chartContainer = document.createElement('div');
    chartContainer.className = 'bg-white p-3 rounded border';
    chartContainer.innerHTML = `
        <h4 class="text-sm font-medium text-gray-700 mb-2">Motor Angles: Current vs Next</h4>
        <div style="height: 250px;">
            <canvas id="motorAnglesChart_${joint}_${dof}"></canvas>
        </div>
    `;
    
    setTimeout(() => {
        const ctx = document.getElementById(`motorAnglesChart_${joint}_${dof}`);
        if (!ctx) return;
        
        const motorAngles = dofData.motor_angles || {};
        const timeData = (motorAngles.agonist_current || []).map((_, idx) => idx);
        
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: timeData,
                datasets: [
                    {
                        label: `${motorNames.agonist} (Corrente)`,
                        data: motorAngles.agonist_current || [],
                        borderColor: 'rgba(255, 99, 132, 1)',
                        backgroundColor: 'rgba(255, 99, 132, 0.1)',
                        borderWidth: 2,
                        fill: false
                    },
                    {
                        label: `${motorNames.antagonist} (Corrente)`,
                        data: motorAngles.antagonist_current || [],
                        borderColor: 'rgba(54, 162, 235, 1)',
                        backgroundColor: 'rgba(54, 162, 235, 0.1)',
                        borderWidth: 2,
                        fill: false
                    },
                    {
                        label: `${motorNames.agonist} (Prossimo)`,
                        data: motorAngles.agonist_next || [],
                        borderColor: 'rgba(255, 99, 132, 0.6)',
                        backgroundColor: 'rgba(255, 99, 132, 0.05)',
                        borderWidth: 1,
                        borderDash: [5, 5],
                        fill: false
                    },
                    {
                        label: `${motorNames.antagonist} (Prossimo)`,
                        data: motorAngles.antagonist_next || [],
                        borderColor: 'rgba(54, 162, 235, 0.6)',
                        backgroundColor: 'rgba(54, 162, 235, 0.05)',
                        borderWidth: 1,
                        borderDash: [5, 5],
                        fill: false
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        title: { display: true, text: 'Campione' }
                    },
                    y: {
                        title: { display: true, text: 'Gradi (°)' }
                    }
                },
                plugins: {
                    legend: { 
                        display: true,
                        position: 'top',
                        labels: { boxWidth: 12, fontSize: 10 }
                    }
                }
            }
        });
        
        // Memorizza il grafico
        const chartKey = `${joint}_${dof}`;
        if (!movementChartsMultiDof[chartKey]) {
            movementChartsMultiDof[chartKey] = {};
        }
        movementChartsMultiDof[chartKey].motorAngles = chart;
    }, 100);
    
    return chartContainer;
}

function createMotorTorquesChart(joint, dof, dofData, motorNames) {
    const chartContainer = document.createElement('div');
    chartContainer.className = 'bg-white p-3 rounded border';
    chartContainer.innerHTML = `
        <h4 class="text-sm font-medium text-gray-700 mb-2">Coppie Motori</h4>
        <div style="height: 200px;">
            <canvas id="motorTorquesChart_${joint}_${dof}"></canvas>
        </div>
    `;
    
    setTimeout(() => {
        const ctx = document.getElementById(`motorTorquesChart_${joint}_${dof}`);
        if (!ctx) return;
        
        const motorTorques = dofData.motor_torques || {};
        const timeData = (motorTorques.agonist || []).map((_, idx) => idx);
        
        const chart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: timeData,
                datasets: [
                    {
                        label: `Coppia ${motorNames.agonist}`,
                        data: motorTorques.agonist || [],
                        borderColor: 'rgba(255, 206, 86, 1)',
                        backgroundColor: 'rgba(255, 206, 86, 0.1)',
                        borderWidth: 2,
                        fill: true
                    },
                    {
                        label: `Coppia ${motorNames.antagonist}`,
                        data: motorTorques.antagonist || [],
                        borderColor: 'rgba(153, 102, 255, 1)',
                        backgroundColor: 'rgba(153, 102, 255, 0.1)',
                        borderWidth: 2,
                        fill: true
                    }
                ]
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                animation: false,
                scales: {
                    x: {
                        title: { display: true, text: 'Campione' }
                    },
                    y: {
                        title: { display: true, text: 'Coppia' }
                    }
                },
                plugins: {
                    legend: { 
                        display: true,
                        position: 'top',
                        labels: { boxWidth: 12 }
                    }
                }
            }
        });
        
        // Memorizza il grafico
        const chartKey = `${joint}_${dof}`;
        if (!movementChartsMultiDof[chartKey]) {
            movementChartsMultiDof[chartKey] = {};
        }
        movementChartsMultiDof[chartKey].motorTorques = chart;
    }, 100);
    
    return chartContainer;
}

function clearAllMovementCharts() {
    // Distruggi tutti i grafici esistenti
    Object.values(movementChartsMultiDof).forEach(dofCharts => {
        Object.values(dofCharts).forEach(chart => {
            if (chart && typeof chart.destroy === 'function') {
                chart.destroy();
            }
        });
    });
    
    // Reset dell'oggetto
    movementChartsMultiDof = {};
    
    // Pulisci il container
    const container = document.getElementById('movementChartsMultiDofContainer');
    if (container) {
        container.innerHTML = `
            <div class="text-center text-gray-500 py-8">
                <i class="fas fa-chart-line text-4xl mb-4"></i>
                <p>No multi-DOF movement data available</p>
                <p class="text-sm">Charts will appear here when data is received from PICO</p>
            </div>
        `;
    }
}

function requestMovementDataFromPico() {
    const joint = $("#jointSelect").val();
    const assignedPort = jointPortMapping[joint];
    
    if (!assignedPort) {
        appendStatusMessage(`⚠️ Associate a serial port to ${joint} before requesting movement data.`);
        return;
    }
    
    appendStatusMessage(`📊 Requesting movement data from ${joint}...`);
    sendCommand('GET_MOVEMENT_DATA');
}

function fetchMovementDataMultiDof(joint = null, dof = null) {
    const url = joint && dof ? 
        `/get_movement_data_multi_dof?joint=${joint}&dof=${dof}` : 
        '/get_movement_data_multi_dof';
        
    $.ajax({
        url: url,
        method: "GET",
        dataType: "json",
        success: function(response) {
            if (response.status === "success" && response.has_data && response.data) {
                renderMovementChartMultiDof(response.data);
                const filterInfo = response.filter ? 
                    ` (filtrato: ${response.filter.joint || 'all'} DOF ${response.filter.dof || 'all'})` : '';
                appendStatusMessage(`Multi-DOF movement data loaded${filterInfo}`);
            } else {
                clearAllMovementCharts();
                appendStatusMessage("No multi-DOF movement data available");
            }
        },
        error: function(xhr, status, error) {
            console.error("Error loading multi-DOF movement data:", error);
            clearAllMovementCharts();
            appendStatusMessage(`Error loading movement data: ${error}`);
        }
    });
}

function updateMovementDataRefreshButton(joint, dof, samples) {
    const refreshButton = document.getElementById('refreshMovementDataBtn');
    if (refreshButton) {
        refreshButton.innerHTML = `
            <i class="fas fa-sync mr-2"></i>Refresh Movement Data
            <span class="text-xs block">${joint} DOF ${dof} (${samples} samples)</span>
        `;
    }
}

function updateMappingDataInfo(info) {
    const infoElement = document.getElementById('mappingDataInfo');
    if (infoElement) {
        infoElement.textContent = info;
    }
}

// Functions for managing saved mapping files

function listSavedMappingFiles() {
    $.ajax({
        url: "/list_saved_mapping_files",
        method: "GET",
        dataType: "json",
        success: function(response) {
            console.log("Saved mapping files:", response);
            if (response.status === "success" && response.count > 0) {
                let message = `Saved mapping files (${response.count}):`;
                response.files.forEach(file => {
                    const timestamp = new Date(file.modified_time * 1000).toLocaleDateString();
                    message += `\n- ${file.joint_name}: ${file.total_points} points, ${file.actual_dof_count} DOF (${timestamp})`;
                });
                appendStatusMessage(message);
            } else {
                appendStatusMessage("No saved mapping files found");
            }
        },
        error: function(xhr, status, error) {
            appendStatusMessage("Error loading mapping file list");
            console.error("Error listing files:", error);
        }
    });
}

function deleteSavedMappingData(jointName) {
    if (!jointName) {
        jointName = $("#jointSelect").val();
    }
    
    if (confirm(`Are you sure you want to delete saved mapping data for ${jointName}?`)) {
        $.ajax({
            url: `/delete_saved_mapping_data/${jointName}`,
            method: "DELETE",
            dataType: "json",
            success: function(response) {
                if (response.status === "success") {
                    appendStatusMessage(`Mapping data deleted for ${jointName}`);
                    // Update chart if we're viewing the deleted joint
                    const currentJoint = $("#jointSelect").val();
                    if (currentJoint === jointName) {
                        clearAllMappingCharts();
                    }
                } else {
                    appendStatusMessage(`Error: ${response.message}`);
                }
            },
            error: function(xhr, status, error) {
                appendStatusMessage(`Error deleting data for ${jointName}`);
                console.error("Deletion error:", error);
            }
        });
    }
}

function saveEnrichedMappingData(jointName, enrichedData) {
    /**
     * Saves enriched mapping data to the server
     * 
     * @param {string} jointName - Joint name
     * @param {object} enrichedData - Data enriched with interpolation and extrapolation
     */
    
    $.ajax({
        url: '/save_enriched_mapping_data',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({
            joint_name: jointName,
            enriched_data: enrichedData
        }),
        success: function(response) {
            if (response.status === 'success') {
                appendStatusMessage(`✅ Enriched data saved for ${jointName}: ${response.total_dofs} DOF with interpolation and extrapolation`);
            } else {
                appendStatusMessage(`❌ Error saving enriched data for ${jointName}: ${response.message}`);
            }
        },
        error: function(xhr, status, error) {
            appendStatusMessage(`❌ Network error saving enriched data for ${jointName}: ${error}`);
            console.error('Error saving enriched data:', error);
        }
    });
}

function refreshMappingDataForCurrentJoint() {
    // Force reload data for current joint
    const selectedJoint = $("#jointSelect").val();
    appendStatusMessage(`Reloading mapping data for ${selectedJoint}...`);
    fetchMappingChartData();
}

// === MULTI-DOF FUNCTIONS ===

/**
 * Sets quick angles for Multi-DOF command
 * If auto-execute toggle is active, immediately executes movement
 */
function setMultiDofQuickAngles(angle0, angle1) {
    $("#multiDofAngle0").val(angle0);
    $("#multiDofAngle1").val(angle1);
    updateMultiDofCommandPreview();
    appendStatusMessage(`Quick angles set: DOF0=${angle0}°, DOF1=${angle1}°`);
    
    // Check if auto-execute toggle is active
    if ($("#autoExecuteToggle").is(":checked")) {
        // Execute movement immediately
        sendMultiDofMove();
        appendStatusMessage(`🚀 Auto-execute active: movement started immediately`);
    }
}

/**
 * Updates Multi-DOF command preview while user modifies parameters
 */
function updateMultiDofCommandPreview() {
    const joint = $("#jointSelect").val();
    const angle0 = parseFloat($("#multiDofAngle0").val()) || 0;
    const angle1 = parseFloat($("#multiDofAngle1").val()) || 0;
    const mask = parseInt($("#multiDofMask").val()) || 3;
    const sync = parseInt($("#multiDofSync").val()) ?? 1;  // Use ?? to allow 0 (SYNC_NONE)
    const speed = parseFloat($("#multiDofSpeed").val()) || 0.5;
    const accel = parseFloat($("#multiDofAccel").val()) || 2.0;
    const path = parseInt($("#multiDofPath").val()) ?? 1;  // Use ?? to allow 0 (PATH_LINEAR)
    
    const command = `${joint}:ALL:MOVE_MULTI_DOF:${angle0}:${angle1}:0:${mask}:${sync}:${speed}:${accel}:${path}`;
    $("#multiDofCommandPreview").val(command);
}

/**
 * Generates quick action buttons based on current joint's interpolated mapping data
 */
function generateSmartQuickButtons() {
    const joint = $("#jointSelect").val();
    const jointType = joint.split('_')[0]; // Extract KNEE, ANKLE or HIP
    
    // Container for buttons
    const container = $('#smartQuickButtons');
    if (!container.length) return;
    
    // If no mapping data available, use default values
    if (!automaticMappingData || !automaticMappingData.present_dofs) {
        generateDefaultQuickButtons(jointType, container);
        return;
    }
    
    // Analyze mapping data to get actual ranges
    const mappingRanges = extractMappingRanges(automaticMappingData, jointType);
    
    if (!mappingRanges) {
        generateDefaultQuickButtons(jointType, container);
        return;
    }
    
    // Generate smart buttons based on mapping ranges
    generateIntelligentQuickButtons(mappingRanges, container);
}

/**
 * Extracts mapping ranges from loaded data
 */
function extractMappingRanges(mappingData, jointType) {
    const ranges = {};
    
    mappingData.present_dofs.forEach(dof => {
        const dofData = mappingData[`dof_${dof}`];
        if (dofData && dofData.joint_angles) {
            // Filter valid angles
            const validAngles = dofData.joint_angles.filter(angle => 
                angle !== null && angle !== undefined && !isNaN(angle)
            );
            
            if (validAngles.length > 0) {
                ranges[dof] = {
                    min: Math.min(...validAngles),
                    max: Math.max(...validAngles),
                    count: validAngles.length
                };
                
                // If interpolation data is available, use extended range
                if (dofData.interpolation && dofData.interpolation.agonist && 
                    dofData.interpolation.agonist.extended_range) {
                    const extendedRange = dofData.interpolation.agonist.extended_range;
                    ranges[dof].extended_min = extendedRange.min;
                    ranges[dof].extended_max = extendedRange.max;
                } else {
                    ranges[dof].extended_min = ranges[dof].min;
                    ranges[dof].extended_max = ranges[dof].max;
                }
            }
        }
    });
    
    return Object.keys(ranges).length > 0 ? ranges : null;
}

/**
 * Generates smart buttons based on mapping ranges
 */
function generateIntelligentQuickButtons(ranges, container) {
    container.empty();
    
    // Check automatic execution toggle state
    const isAutoExecuteEnabled = $("#autoExecuteToggle").is(":checked");
    
    // Header informativo
    container.append(`
        <h4 class="text-sm font-medium text-gray-700 mb-2">
            <i class="fas fa-brain mr-1"></i>Posizioni Intelligenti (basate su mappatura):
            ${isAutoExecuteEnabled ? 
                '<span class="text-orange-500 ml-2"><i class="fas fa-bolt"></i> Auto-execute active</span>' : 
                ''}
        </h4>
    `);
    
    const buttonContainer = $('<div class="grid grid-cols-4 gap-2"></div>');
    
    // Pulsante Zero sempre presente
    buttonContainer.append(`
        <button onclick="setMultiDofQuickAngles(0, 0)" 
                class="bg-gray-500 hover:bg-gray-600 text-white text-xs py-2 px-1 rounded">
            Zero<br><span class="text-xs opacity-75">0°/0°</span>
        </button>
    `);
    
    // Genera combinazioni intelligenti
    const dofs = Object.keys(ranges).map(Number).sort();
    const combinations = generateSmartCombinations(ranges, dofs);
    
    // Limita a 15 pulsanti totali (incluso Zero)
    const maxButtons = 15;
    combinations.slice(0, maxButtons - 1).forEach(combo => {
        const { angle0, angle1, label, type } = combo;
        const colorClass = getButtonColorClass(type);
        
        buttonContainer.append(`
            <button onclick="setMultiDofQuickAngles(${angle0}, ${angle1})" 
                    class="${colorClass} text-white text-xs py-2 px-1 rounded transition-colors"
                    title="${label}${isAutoExecuteEnabled ? ' (esecuzione automatica attiva)' : ''}">
                ${label}<br><span class="text-xs opacity-75">${angle0}°/${angle1}°</span>
            </button>
        `);
    });
    
    container.append(buttonContainer);
    
    // Aggiungi info sui range
    const infoDiv = $('<div class="mt-2 text-xs text-gray-600"></div>');
    dofs.forEach(dof => {
        const range = ranges[dof];
        const originalRange = `${range.min.toFixed(0)}° - ${range.max.toFixed(0)}°`;
        const extendedRange = range.extended_min !== range.min || range.extended_max !== range.max 
            ? ` (esteso: ${range.extended_min.toFixed(0)}° - ${range.extended_max.toFixed(0)}°)` 
            : '';
        
        infoDiv.append(`<div>DOF ${dof}: ${originalRange}${extendedRange}</div>`);
    });
    container.append(infoDiv);
    
    // Apply class for automatic execution if enabled
    if (isAutoExecuteEnabled) {
        container.addClass("auto-execute-enabled");
    } else {
        container.removeClass("auto-execute-enabled");
    }
}

/**
 * Generates smart angle combinations based on ranges
 */
function generateSmartCombinations(ranges, dofs) {
    const combinations = [];
    
    // For each available DOF
    dofs.forEach(dof => {
        const range = ranges[dof];
        
        // Use extended range if available
        const minAngle = range.extended_min;
        const maxAngle = range.extended_max;
        const midAngle = (minAngle + maxAngle) / 2;
        
        // Generate significant points for this DOF
        const significantAngles = [
            minAngle,
            minAngle + (maxAngle - minAngle) * 0.25, // 25%
            midAngle,                                 // 50%
            minAngle + (maxAngle - minAngle) * 0.75, // 75%
            maxAngle
        ].map(angle => Math.round(angle * 10) / 10); // Arrotonda a 1 decimale
        
        // Crea combinazioni per singolo DOF
        significantAngles.forEach((angle, index) => {
            const labels = ['Min', '25%', 'Mid', '75%', 'Max'];
            const types = ['min', 'quarter', 'mid', 'threequarter', 'max'];
            
            if (dof === 0) {
                combinations.push({
                    angle0: angle,
                    angle1: 0,
                    label: `${labels[index]}0`,
                    type: types[index]
                });
            } else if (dof === 1) {
                combinations.push({
                    angle0: 0,
                    angle1: angle,
                    label: `${labels[index]}1`,
                    type: types[index]
                });
            }
        });
    });
    
    // Se abbiamo entrambi i DOF, genera alcune combinazioni coordinate
    if (dofs.includes(0) && dofs.includes(1)) {
        const range0 = ranges[0];
        const range1 = ranges[1];
        
        // Combinazioni coordinate interessanti
        const coordCombinations = [
            {
                angle0: range0.extended_min,
                angle1: range1.extended_min,
                label: 'Min/Min',
                type: 'coord-min'
            },
            {
                angle0: range0.extended_max,
                angle1: range1.extended_max,
                label: 'Max/Max',
                type: 'coord-max'
            },
            {
                angle0: (range0.extended_min + range0.extended_max) / 2,
                angle1: (range1.extended_min + range1.extended_max) / 2,
                label: 'Mid/Mid',
                type: 'coord-mid'
            },
            {
                angle0: range0.extended_max,
                angle1: range1.extended_min,
                label: 'Max/Min',
                type: 'coord-mixed'
            },
            {
                angle0: range0.extended_min,
                angle1: range1.extended_max,
                label: 'Min/Max',
                type: 'coord-mixed'
            }
        ];
        
        coordCombinations.forEach(combo => {
            combo.angle0 = Math.round(combo.angle0 * 10) / 10;
            combo.angle1 = Math.round(combo.angle1 * 10) / 10;
            combinations.push(combo);
        });
    }
    
    // Remove duplicates and sort by relevance
    const uniqueCombinations = combinations.filter((combo, index, arr) => 
        arr.findIndex(c => c.angle0 === combo.angle0 && c.angle1 === combo.angle1) === index
    );
    
    // Priority: coordinates > single DOF > min/max > mid
    const priority = {
        'coord-mid': 1,
        'coord-min': 2, 
        'coord-max': 2,
        'coord-mixed': 3,
        'mid': 4,
        'quarter': 5,
        'threequarter': 5,
        'min': 6,
        'max': 6
    };
    
    return uniqueCombinations.sort((a, b) => 
        (priority[a.type] || 10) - (priority[b.type] || 10)
    );
}

/**
 * Restituisce la classe CSS per il colore del pulsante basata sul tipo
 */
function getButtonColorClass(type) {
    const colorMap = {
        'coord-mid': 'bg-indigo-500 hover:bg-indigo-600',
        'coord-min': 'bg-purple-500 hover:bg-purple-600',
        'coord-max': 'bg-purple-500 hover:bg-purple-600',
        'coord-mixed': 'bg-violet-500 hover:bg-violet-600',
        'mid': 'bg-indigo-400 hover:bg-indigo-500',
        'quarter': 'bg-blue-400 hover:bg-blue-500',
        'threequarter': 'bg-blue-400 hover:bg-blue-500',
        'min': 'bg-teal-400 hover:bg-teal-500',
        'max': 'bg-teal-400 hover:bg-teal-500'
    };
    
    return colorMap[type] || 'bg-indigo-400 hover:bg-indigo-500';
}

/**
 * Generates default buttons when no mapping data is available
 */
function generateDefaultQuickButtons(jointType, container) {
    container.empty();
    
    // Check automatic execution toggle state
    const isAutoExecuteEnabled = $("#autoExecuteToggle").is(":checked");
    
    container.append(`
        <h4 class="text-sm font-medium text-gray-700 mb-2">
            <i class="fas fa-cog mr-1"></i>Posizioni Predefinite (${jointType}):
            ${isAutoExecuteEnabled ? 
                '<span class="text-orange-500 ml-2"><i class="fas fa-bolt"></i> Auto-execute active</span>' : 
                ''}
        </h4>
    `);
    
    const buttonContainer = $('<div class="grid grid-cols-4 gap-2"></div>');
    
    // Default buttons based on joint type
    const defaultButtons = getDefaultButtonsForJoint(jointType);
    
    defaultButtons.forEach(btn => {
        buttonContainer.append(`
            <button onclick="setMultiDofQuickAngles(${btn.angle0}, ${btn.angle1})" 
                    class="${btn.colorClass} text-white text-xs py-2 px-1 rounded"
                    title="${btn.label}${isAutoExecuteEnabled ? ' (esecuzione automatica attiva)' : ''}">
                ${btn.label}<br><span class="text-xs opacity-75">${btn.angle0}°/${btn.angle1}°</span>
            </button>
        `);
    });
    
    container.append(buttonContainer);
    container.append('<div class="mt-2 text-xs text-gray-600">⚠️ No mapping data available - using default values</div>');
    
    // Apply class for automatic execution if enabled
    if (isAutoExecuteEnabled) {
        container.addClass("auto-execute-enabled");
    } else {
        container.removeClass("auto-execute-enabled");
    }
}

/**
 * Returns default buttons for joint type
 */
function getDefaultButtonsForJoint(jointType) {
    const baseColor = 'bg-gray-500 hover:bg-gray-600';
    const defaultColor = 'bg-indigo-400 hover:bg-indigo-500';
    
    const defaults = {
        'KNEE': [
            { angle0: 0, angle1: 0, label: 'Zero', colorClass: baseColor },
            { angle0: 15, angle1: 0, label: '15°', colorClass: defaultColor },
            { angle0: 30, angle1: 0, label: '30°', colorClass: defaultColor },
            { angle0: 45, angle1: 0, label: '45°', colorClass: defaultColor },
            { angle0: 60, angle1: 0, label: '60°', colorClass: defaultColor },
            { angle0: 90, angle1: 0, label: '90°', colorClass: defaultColor }
        ],
        'ANKLE': [
            { angle0: 0, angle1: 0, label: 'Zero', colorClass: baseColor },
            { angle0: 15, angle1: -10, label: '15°/-10°', colorClass: defaultColor },
            { angle0: 30, angle1: 0, label: '30°/0°', colorClass: defaultColor },
            { angle0: -15, angle1: 15, label: '-15°/15°', colorClass: defaultColor },
            { angle0: 20, angle1: -20, label: '20°/-20°', colorClass: defaultColor },
            { angle0: -10, angle1: 25, label: '-10°/25°', colorClass: defaultColor }
        ],
        'HIP': [
            { angle0: 0, angle1: 0, label: 'Zero', colorClass: baseColor },
            { angle0: 20, angle1: 15, label: '20°/15°', colorClass: defaultColor },
            { angle0: 45, angle1: 0, label: '45°/0°', colorClass: defaultColor },
            { angle0: -15, angle1: 30, label: '-15°/30°', colorClass: defaultColor },
            { angle0: 60, angle1: -15, label: '60°/-15°', colorClass: defaultColor },
            { angle0: 30, angle1: 30, label: '30°/30°', colorClass: defaultColor }
        ]
    };
    
    return defaults[jointType] || defaults['KNEE'];
}

/**
 * Sends Multi-DOF command to system
 */
function sendMultiDofMove() {
    const joint = $("#jointSelect").val();
    const angle0 = parseFloat($("#multiDofAngle0").val()) || 0;
    const angle1 = parseFloat($("#multiDofAngle1").val()) || 0;
    const mask = parseInt($("#multiDofMask").val()) || 3;
    const sync = parseInt($("#multiDofSync").val()) ?? 1;  // Use ?? to allow 0 (SYNC_NONE)
    const speed = parseFloat($("#multiDofSpeed").val()) || 0.5;
    const accel = parseFloat($("#multiDofAccel").val()) || 2.0;
    const path = parseInt($("#multiDofPath").val()) ?? 1;  // Use ?? to allow 0 (PATH_LINEAR)
    
    // Validazione input
    if (!joint) {
        alert("Select a joint before sending command");
        return;
    }
    
    // Check angle limits (typical range -180 to +180)
    if (Math.abs(angle0) > 180 || Math.abs(angle1) > 180) {
        if (!confirm("Warning: one or more angles exceed ±180°. Continue?")) {
            return;
        }
    }
    
    // Build command according to MOVE_MULTI_DOF protocol
    const multiDofData = {
        joint: joint,
        angle0: angle0,
        angle1: angle1,
        angle2: 0, // Fixed at 0 for now (DOF 2 not implemented in UI)
        mask: mask,
        sync: sync,
        speed: speed,
        accel: accel,
        path: path
    };
    
    // Update command preview
    updateMultiDofCommandPreview();
    
    // Detailed status message
    const maskBinary = mask.toString(2).padStart(3, '0');
    const activeDofs = [];
    if (mask & 1) activeDofs.push("DOF 0");
    if (mask & 2) activeDofs.push("DOF 1");
    if (mask & 4) activeDofs.push("DOF 2");
    
    const syncMessages = ["no synchronization", "sync by duration", "sync by speed"];
    const pathMessages = ["PATH_LINEAR", "PATH_TRIG", "PATH_QUAD"];
    
    appendStatusMessage(`🚀 Sending Multi-DOF command for ${joint}:`);
    appendStatusMessage(`  • Active DOFs: ${activeDofs.join(", ")} (mask: ${mask} = ${maskBinary}b)`);
    appendStatusMessage(`  • Angles: DOF0=${angle0}°, DOF1=${angle1}°`);
    appendStatusMessage(`  • Strategy: ${syncMessages[sync] || "unknown"}`);
    appendStatusMessage(`  • Speed: ${speed} rad/s, Accel: ${accel}s`);
    appendStatusMessage(`  • Path: ${pathMessages[path] || "unknown"}`);
    
    // Send command
    sendCommand('move-multi-dof', multiDofData);
}

// === MULTI-DOF EVENT INITIALIZATION ===

/**
 * Handles auto-execute toggle state change
 */
function handleAutoExecuteToggle() {
    const isAutoExecuteEnabled = $("#autoExecuteToggle").is(":checked");
    const status = isAutoExecuteEnabled ? "enabled" : "disabled";
    const icon = isAutoExecuteEnabled ? "🚀" : "⏸️";
    
    appendStatusMessage(`${icon} Auto-execute ${status} for smart buttons`);
    
    // Visually update smart buttons to indicate mode
    if (isAutoExecuteEnabled) {
        $("#smartQuickButtons").addClass("auto-execute-enabled");
    } else {
        $("#smartQuickButtons").removeClass("auto-execute-enabled");
    }
}

// Add event handlers when document is ready
$(document).ready(function() {
    // Add event handlers for automatic Multi-DOF command preview update
    $("#multiDofAngle0, #multiDofAngle1, #multiDofMask, #multiDofSync, #multiDofSpeed, #multiDofAccel, #multiDofPath").on('input change', function() {
        updateMultiDofCommandPreview();
    });
    
    // Also update when selected joint changes
    $("#jointSelect").on('change', function() {
        updateMultiDofCommandPreview();
        
    });
    
    // Event listener per il toggle di esecuzione automatica
    $("#autoExecuteToggle").on('change', function() {
        handleAutoExecuteToggle();
    });
    
    // Initialize preview on load (with brief delay)
    setTimeout(updateMultiDofCommandPreview, 200);
    
    // Initialize smart buttons on load
    setTimeout(generateSmartQuickButtons, 250);
    
    // Initialize DOF-specific buttons on load
    renderDofControlButtons();
    
    // Configure listener for intelligent status message scrolling
    const statusMessages = $("#statusMessages");
    let scrollTimeout;
    
    statusMessages.on('scroll', function() {
        const element = this;
        
        // Debounce scroll control for performance
        clearTimeout(scrollTimeout);
        scrollTimeout = setTimeout(function() {
            const isAtBottom = isScrolledToBottom(element);
            
            if (isAtBottom) {
                // User is at bottom - re-enable auto-scroll
                userScrolledUp = false;
                autoScrollEnabled = true;
            } else {
                // User scrolled up - disable auto-scroll
                userScrolledUp = true;
                autoScrollEnabled = false;
            }
            
            // Update visual indicator
            updateScrollIndicator();
        }, 150); // 150ms debounce
    });
});

// Set Zero functions for specific DOF
function sendSetZeroForDof(dof) {
    const dofValue = String(dof);
    const joint = $("#jointSelect").val();
    
    // Get zero_angle_offset from backend data
    const dofInfo = jointPhysicalLimits?.[joint]?.dofs?.find(d => d.index === parseInt(dof));
    const zeroAngleOffset = dofInfo?.zero_angle_offset ?? 0;
    
    // Build confirmation message with explicit angle reference
    let angleMessage = '';
    if (zeroAngleOffset !== 0) {
        angleMessage = `This will set the current position as reference (${zeroAngleOffset.toFixed(1)}°).`;
    } else {
        angleMessage = `This will set the current position as zero reference.`;
    }
    
    // Safety confirmation to prevent accidental clicks
    const confirmed = confirm(
        `Set ZERO for ${joint} DOF ${dofValue}?\n\n${angleMessage}`
    );
    
    if (!confirmed) {
        appendStatusMessage(`Set Zero cancelled for ${joint} DOF ${dofValue}`);
        return;
    }
    
    sendCommand('set-zero', { dof: dofValue });
    appendStatusMessage(`Set Zero sent for ${joint} DOF ${dofValue}`);
}

function sendRecalcOffsetForDof(dof) {
    const dofValue = String(dof);
    sendCommand('recalc-offset', { dof: dofValue });
    const joint = $("#jointSelect").val();
    appendStatusMessage(`Recalc Offset sent for ${joint} DOF ${dofValue}`);
}

function renderDofControlButtons() {
    Object.entries(JOINT_DOF_UI_CONFIG).forEach(([jointType, dofConfigs]) => {
        dofConfigs.forEach(dofConfig => {
            const container = $(`#${dofConfig.containerId}`);
            if (!container.length) return;

            container.empty();

            // Pretension button (green)
            const pretensionButton = $(`
                <button class="w-full text-sm text-white py-2 px-3 rounded font-medium bg-green-500 hover:bg-green-600">
                    <i class="fas fa-compress-arrows-alt mr-2"></i>${dofConfig.pretensionLabel}
                </button>
            `);
            pretensionButton.on('click', () => sendPretension(dofConfig.dof));

            // Release button (red)
            const releaseButton = $(`
                <button class="w-full text-sm text-white py-2 px-3 rounded font-medium bg-red-500 hover:bg-red-600">
                    <i class="fas fa-expand-arrows-alt mr-2"></i>${dofConfig.releaseLabel}
                </button>
            `);
            releaseButton.on('click', () => sendRelease(dofConfig.dof));

            // Get zero_angle_offset from backend data
            const currentJoint = $("#jointSelect").val();
            const dofInfo = jointPhysicalLimits?.[currentJoint]?.dofs?.find(d => d.index === parseInt(dofConfig.dof));
            const zeroAngleOffset = dofInfo?.zero_angle_offset ?? 0;
            
            // Debug log to verify data
            if (dofInfo) {
                console.log(`[renderDofControlButtons] ${currentJoint} DOF ${dofConfig.dof}: zero_angle_offset=${zeroAngleOffset}°`);
            } else {
                console.warn(`[renderDofControlButtons] No DOF info found for ${currentJoint} DOF ${dofConfig.dof}`);
            }
            
            // Set Zero button (yellow/orange depending on DOF)
            // Show the reference angle that will be set
            const zeroAngleText = zeroAngleOffset !== 0 ? ` (${zeroAngleOffset.toFixed(1)}°)` : '';
            const setZeroButton = $(`
                <button class="w-full text-sm text-white py-2 px-3 rounded font-medium ${dofConfig.setZeroClasses}">
                    <i class="fas fa-map-marker-alt mr-2"></i>${dofConfig.setZeroLabel}${zeroAngleText}
                    <br><span class="text-xs opacity-80">${dofConfig.setZeroSubtitle}</span>
                </button>
            `);
            setZeroButton.on('click', () => sendSetZeroForDof(dofConfig.dof));

            // Recalc Offset button (purple)
            const recalcButton = $(`
                <button class="w-full text-sm text-white py-2 px-3 rounded font-medium bg-purple-500 hover:bg-purple-600">
                    <i class="fas fa-sync-alt mr-2"></i>${dofConfig.recalcLabel}
                </button>
            `);
            recalcButton.on('click', () => sendRecalcOffsetForDof(dofConfig.dof));

            // Append in order: Pretension → Release → Set Zero → Recalc Offset
            container.append(pretensionButton, releaseButton, setZeroButton, recalcButton);
        });
    });
}

function getPhysicalLimitsForJoint(jointName, dofIndex) {
    if (!jointName && jointName !== 0) return null;
    const limitsForJoint = jointPhysicalLimits?.[jointName];
    if (!limitsForJoint) return null;

    const key = String(dofIndex);
    const minValue = limitsForJoint.min ? Number(limitsForJoint.min[key]) : undefined;
    const maxValue = limitsForJoint.max ? Number(limitsForJoint.max[key]) : undefined;

    if (!Number.isFinite(minValue) || !Number.isFinite(maxValue)) {
        return null;
    }

    return { min: minValue, max: maxValue };
}

function computeJointSafeRange(jointName, dofIndex, jointAngles) {
    if (!Array.isArray(jointAngles) || jointAngles.length === 0) return null;

    const numericAngles = jointAngles.filter(angle => Number.isFinite(angle));
    if (numericAngles.length === 0) return null;

    const jointMin = Math.min(...numericAngles);
    const jointMax = Math.max(...numericAngles);
    const jointRange = jointMax - jointMin;

    const MAPPING_EXTENSION_RATIO = 0.10; // stesso valore del firmware
    const PHYSICAL_SAFETY_MARGIN = 1.0;    // margine sui limiti fisici

    const extendedMin = jointMin - jointRange * MAPPING_EXTENSION_RATIO;
    const extendedMax = jointMax + jointRange * MAPPING_EXTENSION_RATIO;

    const physicalLimits = getPhysicalLimitsForJoint(jointName, dofIndex);
    let physicalMin = Number.NEGATIVE_INFINITY;
    let physicalMax = Number.POSITIVE_INFINITY;

    if (physicalLimits) {
        physicalMin = physicalLimits.min + PHYSICAL_SAFETY_MARGIN;
        physicalMax = physicalLimits.max - PHYSICAL_SAFETY_MARGIN;
    }

    let safeMin = Math.max(extendedMin, physicalMin);
    let safeMax = Math.min(extendedMax, physicalMax);

    if (!Number.isFinite(safeMin) || !Number.isFinite(safeMax) || safeMin >= safeMax) {
        safeMin = Number.isFinite(physicalMin) ? physicalMin : jointMin;
        safeMax = Number.isFinite(physicalMax) ? physicalMax : jointMax;
    }

    if (!Number.isFinite(safeMin) || !Number.isFinite(safeMax) || safeMin >= safeMax) {
        safeMin = jointMin;
        safeMax = jointMax;
    }

    return {
        min: safeMin,
        max: safeMax,
        originalMin: jointMin,
        originalMax: jointMax
    };
}

function generateLinearPointsWithinRange(regression, minX, maxX, step = 1) {
    if (!regression || !Number.isFinite(minX) || !Number.isFinite(maxX) || minX > maxX) {
        return [];
    }

    const effectiveStep = Math.max(step || 0, 0.1);
    const span = maxX - minX;
    const totalSteps = span > 0 ? Math.ceil(span / effectiveStep) : 1;
    const points = [];

    for (let i = 0; i <= totalSteps; i++) {
        const isLast = i === totalSteps;
        const xValue = isLast ? maxX : minX + i * effectiveStep;
        const yValue = regression.slope * xValue + regression.intercept;
        points.push({ x: xValue, y: yValue });
    }

    return points;
}

function fetchJointPhysicalLimits() {
    return $.ajax({
        url: '/joint_limits',
        method: 'GET',
        dataType: 'json'
    }).done(response => {
        if (response && response.limits) {
            jointPhysicalLimits = response.limits;
        } else {
            jointPhysicalLimits = {};
        }
    }).fail((xhr, status, error) => {
        jointPhysicalLimits = {};
        appendStatusMessage(`⚠️ Impossibile recuperare i limiti fisici dal backend: ${error || status}`);
    });
}



// === ENCODER TEST FUNCTIONS ===

/**
 * Starts encoder test for specified joint type
 * @param {string} jointType - KNEE, ANKLE, or HIP
 */
function startEncoderTest(jointType) {
    // Stop any active test
    stopEncoderTest();
    
    encoderTestActive = true;
    currentEncoderJointType = jointType;
    
    // Get selected interval for this joint
    const intervalSelector = `#${jointType.toLowerCase()}EncoderInterval`;
    const intervalMs = parseInt($(intervalSelector).val()) || 500;
    
    // Get currently selected full joint
    const selectedJoint = $("#jointSelect").val();
    const jointPrefix = selectedJoint.split('_')[0]; // KNEE, ANKLE, or HIP
    
    // Verify that joint type matches
    if (jointPrefix !== jointType) {
        appendStatusMessage(`⚠️ Error: Encoder test for ${jointType} but selected joint is ${jointPrefix}`);
        return;
    }
    
    // Reset encoder data
    encoderTestData = {
        timestamps: [],
        dofData: {}
    };
    
    // Initialize data for joint DOF
    const dofCount = getDofCountForJoint(jointType);
    for (let i = 0; i < dofCount; i++) {
        encoderTestData.dofData[i] = {
            timestamps: [],
            values: []
        };
    }
    
    // Send start command to backend
    sendCommand('start-test-encoder');
    
    // Start periodic polling
    encoderTestInterval = setInterval(() => {
        if (encoderTestActive) {
            requestEncoderData();
        }
    }, intervalMs);
    
    appendStatusMessage(`🔄 Encoder test started for ${jointType} (interval: ${intervalMs}ms)`);
    
    // Aggiorna UI
    updateEncoderTestUI(true, jointType);
}

/**
 * Stops encoder test
 * @param {string} jointType - Optional, if specified checks that it matches active test
 */
function stopEncoderTest(jointType = null) {
    if (!encoderTestActive) return;
    
    // If jointType specified, verify it matches
    if (jointType && jointType !== currentEncoderJointType) {
        return; // Don't stop if different type
    }
    
    encoderTestActive = false;
    
    // Ferma il polling
    if (encoderTestInterval) {
        clearInterval(encoderTestInterval);
        encoderTestInterval = null;
    }
    
    // Send stop command to backend
    sendCommand('stop-test-encoder');
    
    appendStatusMessage(`⏹️ Encoder test stopped for ${currentEncoderJointType}`);
    
    // Aggiorna UI
    updateEncoderTestUI(false, currentEncoderJointType);
    
    currentEncoderJointType = null;
}

/**
 * Requests current encoder data from backend
 */
function requestEncoderData() {
    const selectedJoint = $("#jointSelect").val();
    
    // Request encoder data via AJAX
    $.ajax({
        url: "/get_encoder_data",
        method: "GET",
        dataType: "json",
        data: { joint: selectedJoint },
        success: function(response) {
            if (response.status === "success" && response.data) {
                updateEncoderDisplay(response.data);
            }
        },
        error: function(xhr, status, error) {
            console.warn("Error in encoder data request:", error);
        }
    });
}

/**
 * Updates encoder display (textual and graphical)
 * @param {object} data - Encoder data received from backend
 */
function updateEncoderDisplay(data) {
    if (!encoderTestActive || !currentEncoderJointType) return;
    
    const timestamp = Date.now();
    const jointType = currentEncoderJointType.toLowerCase();
    
    // Update historical data
    encoderTestData.timestamps.push(timestamp);
    
    // Update textual display for each DOF
    Object.keys(data.dof_positions || {}).forEach(dof => {
        const dofIndex = parseInt(dof);
        const position = data.dof_positions[dof];
        
        // Update textual display
        const valueSpan = $(`#${jointType}EncoderDof${dofIndex}`);
        if (valueSpan.length) {
            valueSpan.text(`${position.toFixed(2)} °`);
            
            // Blinking effect to indicate update
            valueSpan.addClass('bg-yellow-200').removeClass('bg-yellow-200', 300);
        }
        
        // Save in historical data
        if (encoderTestData.dofData[dofIndex]) {
            encoderTestData.dofData[dofIndex].timestamps.push(timestamp);
            encoderTestData.dofData[dofIndex].values.push(position);
            
            // Keep only last 100 points for performance
            if (encoderTestData.dofData[dofIndex].values.length > 100) {
                encoderTestData.dofData[dofIndex].timestamps.shift();
                encoderTestData.dofData[dofIndex].values.shift();
            }
        }
    });
    
    // Update joint-specific chart
    updateJointSpecificChart(currentEncoderJointType, data);
    
    // Keep only last 100 global timestamps
    if (encoderTestData.timestamps.length > 100) {
        encoderTestData.timestamps.shift();
    }
}

/**
 * Updates joint-specific chart with encoder data
 * @param {string} jointType - KNEE, ANKLE, or HIP
 * @param {object} data - Encoder data
 */
function updateJointSpecificChart(jointType, data) {
    if (jointType === 'KNEE') {
        updateKneeChartWithEncoder(data);
    } else if (jointType === 'ANKLE') {
        updateAnkleChartWithEncoder(data);
    } else if (jointType === 'HIP') {
        updateHipChartWithEncoder(data);
    }
}

/**
 * Updates knee chart with encoder data
 */
function updateKneeChartWithEncoder(data) {
    if (!kneeChart || !data.dof_positions) return;
    
    const dof0Position = data.dof_positions['0'];
    if (dof0Position !== undefined) {
        // Check if dataset for encoder already exists
        let encoderDatasetIndex = kneeChart.data.datasets.findIndex(dataset => dataset.label === 'Encoder DOF 0');
        
        if (encoderDatasetIndex === -1) {
            // Add encoder dataset if it doesn't exist
            kneeChart.data.datasets.push({
                label: 'Encoder DOF 0',
                data: [],
                borderColor: 'rgba(255, 206, 86, 1)',
                backgroundColor: 'rgba(255, 206, 86, 0.1)',
                borderWidth: 2,
                fill: false,
                pointRadius: 0,
                pointHoverRadius: 3
            });
            encoderDatasetIndex = kneeChart.data.datasets.length - 1;
        }
        
        // Usa la stessa logica temporale degli altri grafici
        addTimePointToChart(kneeChart, 'knee', dof0Position);
    }
}

/**
 * Updates ankle temporal charts with encoder data
 */
function updateAnkleChartWithEncoder(data) {
    if (!data.dof_positions) return;
    
    const dof0Position = data.dof_positions['0']; // Plantare-Dorsale
    const dof1Position = data.dof_positions['1']; // Inversione-Eversione
    
    // Aggiorna DOF 0 se presente
    if (dof0Position !== undefined && ankleDof0Chart) {
        addTimePointToChart(ankleDof0Chart, 'ankle_dof0', dof0Position);
    }
    
    // Aggiorna DOF 1 se presente
    if (dof1Position !== undefined && ankleDof1Chart) {
        addTimePointToChart(ankleDof1Chart, 'ankle_dof1', dof1Position);
    }
}

/**
 * Updates hip temporal charts with encoder data
 */
function updateHipChartWithEncoder(data) {
    if (!data.dof_positions) return;
    
    const dof0Position = data.dof_positions['0']; // Flessione-Estensione
    const dof1Position = data.dof_positions['1']; // Abduzione-Adduzione
    
    // Aggiorna DOF 0 se presente
    if (dof0Position !== undefined && hipDof0Chart) {
        addTimePointToChart(hipDof0Chart, 'hip_dof0', dof0Position);
    }
    
    // Aggiorna DOF 1 se presente
    if (dof1Position !== undefined && hipDof1Chart) {
        addTimePointToChart(hipDof1Chart, 'hip_dof1', dof1Position);
    }
}

/**
 * Updates encoder test controls UI
 * @param {boolean} isActive - Whether test is active
 * @param {string} jointType - Tipo di giunto
 */
function updateEncoderTestUI(isActive, jointType) {
    if (!jointType) return;
    
    const jointTypeLower = jointType.toLowerCase();
    const startButton = $(`button[onclick="startEncoderTest('${jointType}')"]`);
    const stopButton = $(`button[onclick="stopEncoderTest('${jointType}')"]`);
    const intervalSelect = $(`#${jointTypeLower}EncoderInterval`);
    
    if (isActive) {
        startButton.prop('disabled', true).removeClass('bg-green-500 hover:bg-green-600').addClass('bg-gray-400');
        stopButton.prop('disabled', false).removeClass('bg-gray-400').addClass('bg-red-500 hover:bg-red-600');
        intervalSelect.prop('disabled', true);
    } else {
        startButton.prop('disabled', false).removeClass('bg-gray-400').addClass('bg-green-500 hover:bg-green-600');
        stopButton.prop('disabled', false).removeClass('bg-red-500 hover:bg-red-600').addClass('bg-gray-400');
        intervalSelect.prop('disabled', false);
        
        // Reset value display
        $(`#${jointTypeLower}EncoderValues span`).text('-- °');
    }
}

/**
 * Returns number of DOFs for a joint type
 * @param {string} jointType - KNEE, ANKLE, or HIP
 * @returns {number} Number of DOFs
 */
function getDofCountForJoint(jointType) {
    switch (jointType) {
        case 'KNEE': return 1;
        case 'ANKLE': return 2;
        case 'HIP': return 2;
        default: return 1;
    }
}

/**
 * Reset encoder interface for all joints
 */
function resetAllEncoderUI() {
    const jointTypes = ['KNEE', 'ANKLE', 'HIP'];
    
    jointTypes.forEach(jointType => {
        updateEncoderTestUI(false, jointType);
    });
}
