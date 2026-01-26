// Global variables
let kneeChart;
let ankleDof0Chart, ankleDof1Chart; // Charts for ankle
let hipDof0Chart, hipDof1Chart;     // Charts for hip
let mappingChart;
let mappingCharts = {}; // Object to manage multiple charts for DOFs
let pidErrorChart, pidTorqueChart;  // PID diagnostics charts
let pidDiagStreamActive = false;    // PID diagnostics streaming state
let pidDiagDataBuffer = [];         // Buffer for CSV export (all data, no limit)
let intervalId;
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

// Movement Sequence Builder variables
let movementSequence = []; // Array of {type: "move", dof0, dof1} or {type: "pause", duration}
let isSequencePlaying = false;
let sequencePlaybackHandle = null;
let sequenceExecutionData = []; // Collected during playback: {stepIndex, targetDof0, targetDof1, startTimestamp, endTimestamp, encoderSamples[]}
let isAddToSequenceMode = false;

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

// Joint configuration from joint_config.json
let jointConfigData = null;

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
    
    // Load joint configuration for auto-mapping grid visualization
    fetchJointConfig();
    
    // Fetch serial port configuration first, then initialize joint selection
    fetchSerialPortConfiguration().done(function() {
        // Now that jointPortMapping is populated, we can safely select the joint
        const initialJoint = $("#jointSelect").val();
        sendCommand('select-joint', { joint: initialJoint });
        console.log('Initial PID values requested for joint:', initialJoint);
    });
    
    // Fetch CAN interfaces after serial port configuration
    fetchCanInterfaces({ showStatus: false });
    startCanStatusPolling();

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
        
        // Update auto-mapping progress UI
        updateAutoMappingProgress(data);
        
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

    // Listener for real-time data from sockets (used for sequence playback)
    socket.on('joint_measure', function(data) {
        // Handle sequence playback data collection
        if (isSequencePlaying && sequenceExecutionData.length > 0) {
            const currentExecution = sequenceExecutionData[sequenceExecutionData.length - 1];
            
            // Collect encoder sample with all available data
            currentExecution.encoderSamples.push({
                timestamp: Date.now(),
                serverTimestamp: data.timestamp,
                joint: data.joint,
                dof: data.dof,
                measurements: data.data, // Raw measurement data
                jointAngles: data.joint_angles || null, // DOF positions if available
                motorPositions: data.motor_positions || null // Raw motor data if available
            });
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

    // Listener for CAN encoder stream data (real-time via SocketIO)
    socket.on('encoder_stream', function(data) {
        // Only process if encoder test is active
        if (!encoderTestActive || !currentEncoderJointType) return;
        
        // Filter by joint: only process data from the currently selected joint
        // joint_name from server is like "ANKLE_RIGHT", currentEncoderJointType is like "ankle"
        if (data.joint_name) {
            const expectedPrefix = currentEncoderJointType.toUpperCase();
            if (!data.joint_name.toUpperCase().startsWith(expectedPrefix)) {
                return;  // Ignore data from other joints
            }
        }
        
        // Update the chart and display from real-time CAN data
        if (data && data.angles_deg) {
            updateEncoderChartFromCanStream(data);
            
            // Also store in data buffer for history
            const timestamp = Date.now();
            encoderTestData.timestamps.push(timestamp);
            
            data.angles_deg.forEach((angle, dofIndex) => {
                if (angle !== null && encoderTestData.dofData[dofIndex]) {
                    encoderTestData.dofData[dofIndex].timestamps.push(timestamp);
                    encoderTestData.dofData[dofIndex].values.push(angle);
                }
            });
            
            // Limit buffer size
            const maxDataPoints = 1000;
            while (encoderTestData.timestamps.length > maxDataPoints) {
                encoderTestData.timestamps.shift();
            }
            Object.values(encoderTestData.dofData).forEach(dofData => {
                while (dofData.timestamps.length > maxDataPoints) {
                    dofData.timestamps.shift();
                    dofData.values.shift();
                }
            });
        }
    });

    // Listener for holding target updates (when DOF enters HOLDING mode)
    socket.on('holding_target', function(data) {
        if (data && data.dof !== undefined && data.angle !== undefined) {
            // Update holding target display for this DOF
            updateHoldingTargetDisplay(data.dof, data.angle);
        }
    });

    // Listener for PID diagnostics data (target/error)
    // Temporary storage for combining pid_diag and pid_torque into single records
    let pendingPidRecord = null;
    
    socket.on('pid_diag', function(data) {
        if (!pidDiagStreamActive) return;
        
        // Filter by currently selected joint to avoid flickering from other controllers
        const selectedJoint = $('#jointSelect').val();
        if (data.joint_name && data.joint_name.toUpperCase() !== selectedJoint.toUpperCase()) {
            return;  // Ignore data from other joints
        }
        
        updatePidDiagDisplay(data);
        updatePidErrorChart(data);
        
        // Store for CSV export - create new record with target/error
        const timeSeconds = (Date.now() - pidDiagStartTime) / 1000;
        pendingPidRecord = {
            time_s: timeSeconds.toFixed(3),
            target_dof0: data.target_deg ? data.target_deg[0] : 0,
            target_dof1: data.target_deg ? data.target_deg[1] : 0,
            error_dof0: data.error_deg ? data.error_deg[0] : 0,
            error_dof1: data.error_deg ? data.error_deg[1] : 0,
            current_dof0: data.target_deg && data.error_deg ? (data.target_deg[0] - data.error_deg[0]) : 0,
            current_dof1: data.target_deg && data.error_deg ? (data.target_deg[1] - data.error_deg[1]) : 0,
            torque_a_dof0: 0,
            torque_b_dof0: 0,
            torque_a_dof1: 0,
            torque_b_dof1: 0
        };
    });

    // Listener for PID torque data
    socket.on('pid_torque', function(data) {
        if (!pidDiagStreamActive) return;
        
        // Filter by currently selected joint to avoid flickering from other controllers
        const selectedJoint = $('#jointSelect').val();
        if (data.joint_name && data.joint_name.toUpperCase() !== selectedJoint.toUpperCase()) {
            return;  // Ignore data from other joints
        }
        
        updatePidTorqueDisplay(data);
        updatePidTorqueChart(data);
        
        // Complete the pending record with torque data and add to buffer
        if (pendingPidRecord) {
            pendingPidRecord.torque_a_dof0 = data.torque_A ? data.torque_A[0] : 0;
            pendingPidRecord.torque_b_dof0 = data.torque_B ? data.torque_B[0] : 0;
            pendingPidRecord.torque_a_dof1 = data.torque_A ? data.torque_A[1] : 0;
            pendingPidRecord.torque_b_dof1 = data.torque_B ? data.torque_B[1] : 0;
            
            pidDiagDataBuffer.push(pendingPidRecord);
            pendingPidRecord = null;
            
            // Update buffer count display (throttled)
            if (pidDiagDataBuffer.length % 10 === 0) {
                $('#pidDiagBufferCount').text(pidDiagDataBuffer.length);
            }
        }
    });

    // Listener for movement metrics (received when DOF enters HOLDING)
    socket.on('movement_metrics', function(data) {
        // Filter by currently selected joint
        const selectedJoint = $('#jointSelect').val();
        if (data.joint_name && data.joint_name.toUpperCase() !== selectedJoint.toUpperCase()) {
            return;  // Ignore metrics from other joints
        }
        
        console.log('Movement metrics received:', data);
        updateMovementMetricsDisplay(data);
    });

    // CAN control handlers
    $("#connectCanBtn").on('click', connectCanInterface);
    $("#disconnectCanBtn").on('click', disconnectCanInterface);
    $("#sendCanTimeSync").on('click', sendCanTimeSyncCommand);
    $("#sendCanWaypointBtn").on('click', sendCanWaypointCommand);
    $("#sendMultiWaypointSmoothBtn").on('click', sendMultiWaypointSmoothCurve);
    $("#sendCanWaypointSequenceBtn").on('click', sendCanWaypointSequence);
    $("#sendCosineOscillationBtn").on('click', sendCosineOscillation);
    $("#sendCanEmergency").on('click', sendCanEmergencyStop);

    // Initialize charts
    initializeCharts();
    
    // Handle joint change
    $("#jointSelect").change(function() {
        const joint = $(this).val();
        updateSerialPortSelectUI(joint);
        
        // Update CAN Motion Control panel (joint label + DOF options)
        updateCanMotionJoint();
        
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
        
        // Update oscillation test and sinusoid defaults for the new joint
        updateOscillationDefaults(joint);
        
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
        
        // Clear movement sequence when changing joint (sequence is joint-specific)
        if (movementSequence.length > 0) {
            clearSequence(true); // Silent clear
            appendStatusMessage("🔄 Movement sequence cleared (joint changed)");
        }
        
        // Update sequence builder visibility (only for ANKLE and KNEE)
        updateSequenceBuilderVisibility(joint);
        
        // Show expected mapping grid for new joint
        showExpectedMappingGrid(joint);
    });

    // DOF selection removed - now per-DOF controls handle specific DOF operations
    
    // Toggle mutual exclusion logic for auto-execute and sequence mode
    $("#autoExecuteToggle").change(function() {
        if ($(this).is(":checked")) {
            // Disable sequence mode when auto-execute is enabled
            $("#sequenceModeToggle").prop("checked", false);
            isAddToSequenceMode = false;
            // Hide sequence builder and update visibility
            const currentJoint = $("#jointSelect").val();
            updateSequenceBuilderVisibility(currentJoint);
            // Add auto-execute visual indicator
            $("#smartQuickButtons").addClass("auto-execute-enabled");
            appendStatusMessage("⚡ Auto-execute mode enabled");
        } else {
            // Remove auto-execute visual indicator
            $("#smartQuickButtons").removeClass("auto-execute-enabled");
            appendStatusMessage("⏸️ Auto-execute mode disabled");
        }
    });
    
    $("#sequenceModeToggle").change(function() {
        if ($(this).is(":checked")) {
            // Disable auto-execute when sequence mode is enabled
            $("#autoExecuteToggle").prop("checked", false);
            isAddToSequenceMode = true;
            // Remove auto-execute visual indicator
            $("#smartQuickButtons").removeClass("auto-execute-enabled");
            // Update sequence builder visibility
            const currentJoint = $("#jointSelect").val();
            updateSequenceBuilderVisibility(currentJoint);
            appendStatusMessage("📝 Sequence mode enabled - click grid buttons to build sequence");
        } else {
            isAddToSequenceMode = false;
            // Update sequence builder visibility (will hide)
            const currentJoint = $("#jointSelect").val();
            updateSequenceBuilderVisibility(currentJoint);
            appendStatusMessage("Sequence mode disabled");
        }
    });
    
    // Initialize sequence builder visibility based on initial joint
    updateSequenceBuilderVisibility($("#jointSelect").val());

    $("#serialPortSelect").change(function() {
        const joint = $("#jointSelect").val();
        const port = $(this).val() || null;
        assignSerialPortToJoint(joint, port);
    });

    $("#refreshSerialPorts").on('click', function() {
        fetchSerialPortConfiguration({ showStatus: true });
    });

    $("#canInterfaceSelect").change(function() {
        selectedCanInterface = $(this).val() || null;
        updateCanInterfaceHint();
        if (selectedCanInterface) {
            try {
                const config = JSON.parse(selectedCanInterface);
                appendStatusMessage(`🔌 CAN interface selected: ${config.interface} on ${config.channel}`);
            } catch (e) {
                appendStatusMessage(`⚠️ Invalid CAN interface configuration`);
            }
        } else {
            appendStatusMessage(`🔌 CAN interface deselected`);
        }
    });

    $("#refreshCanInterfaces").on('click', function() {
        fetchCanInterfaces({ showStatus: true });
    });

    $("#testCanInit").on('click', function() {
        testCanInitialization();
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
    
    // Update sinusoid parameters for the selected joint
    updateSinusoidParamsForJoint(joint);
}

/**
 * Update oscillation test defaults based on selected joint
 * Sets appropriate min/max angles for both Oscillation Test and Sinusoid Test sections
 */
function updateOscillationDefaults(jointName) {
    if (!jointName) return;
    
    const jointType = jointName.split('_')[0].toUpperCase();
    
    // Define sensible defaults for each joint type (restricted range within safe limits)
    const jointDefaults = {
        'KNEE': { min: 20, max: 80 },      // Knee: 0-100° safe, use 20-80°
        'ANKLE': { min: -15, max: 15 },    // Ankle: typically -30° to +30°, use -15 to +15°
        'HIP': { min: -20, max: 20 }       // Hip: varies, use -20 to +20°
    };
    
    const defaults = jointDefaults[jointType] || { min: 20, max: 80 };
    
    // Update Oscillation Test (PID Tuning) section
    $('#oscTestPointA').val(defaults.min);
    $('#oscTestPointB').val(defaults.max);
    
    // Update Sinusoidal Trajectory Test section (DOF 0)
    $('#sinusoidDof0Min').val(defaults.min);
    $('#sinusoidDof0Max').val(defaults.max);
    
    // Also update DOF 1 if present (for multi-DOF joints like ankle)
    if ($('#sinusoidDof1Min').length) {
        $('#sinusoidDof1Min').val(defaults.min);
        $('#sinusoidDof1Max').val(defaults.max);
    }
}

/**
 * Update sinusoid parameters UI based on selected joint configuration
 * Preloads min/max values from joint_config and shows/hides DOF containers
 */
function updateSinusoidParamsForJoint(jointName) {
    if (!jointName) return;
    
    // Get joint config from jointConfigData (loaded from server via fetchJointConfig)
    const jointKey = jointName.toLowerCase();
    const jointConfig = jointConfigData?.joints?.[jointKey];
    
    if (!jointConfig) {
        console.warn('Joint config not found for:', jointKey);
        return;
    }
    
    const dofCount = jointConfig.dof_count || 1;
    const dofs = jointConfig.dofs || [];
    
    // Show/hide DOF containers based on dof_count
    $("#sinusoidDof1Container").toggle(dofCount >= 2);
    $("#sinusoidDof2Container").toggle(dofCount >= 3);
    
    // Update each DOF with values from config
    dofs.forEach((dof, index) => {
        const prefix = `#sinusoidDof${index}`;
        
        // Update DOF name
        $(`${prefix}Name`).text(dof.name?.replace(/_/g, '-') || `DOF ${index}`);
        
        // Calculate sensible oscillation range (use auto_mapping range or 50% of full range)
        let minOsc, maxOsc;
        if (dof.auto_mapping_min_angle !== undefined && dof.auto_mapping_max_angle !== undefined) {
            // Use auto-mapping range as default oscillation range
            minOsc = dof.auto_mapping_min_angle;
            maxOsc = dof.auto_mapping_max_angle;
        } else {
            // Use 50% of full range centered on middle
            const center = (dof.min_angle + dof.max_angle) / 2;
            const halfRange = (dof.max_angle - dof.min_angle) / 4;
            minOsc = center - halfRange;
            maxOsc = center + halfRange;
        }
        
        $(`${prefix}Min`).val(Math.round(minOsc));
        $(`${prefix}Max`).val(Math.round(maxOsc));
        
        // Set DOF 0 active by default, others unchecked
        $(`${prefix}Active`).prop('checked', index === 0);
    });
    
    // Update stats display
    updateSinusoidStats();
}

function initializeCharts() {
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
                    title: { 
                        display: true, 
                        text: 'Time (s)',
                        font: { size: 10 }
                    },
                    ticks: { 
                        font: { size: 9 }
                    },
                    grid: {
                        display: true,
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

    // PID Diagnostics Charts
    const pidErrorCtx = document.getElementById('pidErrorChart');
    if (pidErrorCtx) {
        pidErrorChart = new Chart(pidErrorCtx.getContext('2d'), {
            type: 'line',
            data: {
                datasets: [
                    { label: 'DOF 0 Error', data: [], borderColor: 'rgba(255, 99, 132, 1)', borderWidth: 2, fill: false, pointRadius: 0 },
                    { label: 'DOF 1 Error', data: [], borderColor: 'rgba(54, 162, 235, 1)', borderWidth: 2, fill: false, pointRadius: 0 }
                ]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (s)' }, min: 0, max: 30 },
                    y: { title: { display: true, text: 'Error (°)' } }
                },
                plugins: { legend: { position: 'top' } }
            }
        });
    }

    const pidTorqueCtx = document.getElementById('pidTorqueChart');
    if (pidTorqueCtx) {
        pidTorqueChart = new Chart(pidTorqueCtx.getContext('2d'), {
            type: 'line',
            data: {
                datasets: [
                    { label: 'DOF 0 Torque A', data: [], borderColor: 'rgba(75, 192, 192, 1)', borderWidth: 2, fill: false, pointRadius: 0 },
                    { label: 'DOF 0 Torque B', data: [], borderColor: 'rgba(255, 159, 64, 1)', borderWidth: 2, fill: false, pointRadius: 0 },
                    { label: 'DOF 1 Torque A', data: [], borderColor: 'rgba(153, 102, 255, 1)', borderWidth: 2, fill: false, pointRadius: 0, borderDash: [5,5] },
                    { label: 'DOF 1 Torque B', data: [], borderColor: 'rgba(255, 99, 132, 1)', borderWidth: 2, fill: false, pointRadius: 0, borderDash: [5,5] }
                ]
            },
            options: {
                responsive: true,
                animation: false,
                scales: {
                    x: { type: 'linear', position: 'bottom', title: { display: true, text: 'Time (s)' }, min: 0, max: 30 },
                    y: { title: { display: true, text: 'Torque' } }
                },
                plugins: { legend: { position: 'top' } }
            }
        });
    }

    // Initialize multi-DOF mapping charts system
    initializeMappingChartsSystem();
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
                    text: `DOF ${dof} - Interpolation + Extrapolation (Operating Range)`,
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

function generateInterpolatedAndExtrapolatedPoints(regression, minX, maxX, stepSize = 1, operatingLimits = null) {
    /**
     * Generates interpolated and extrapolated points using regression equation
     * Uses operating limits from config if available, otherwise falls back to 10% extrapolation
     * 
     * @param {object} regression - Linear regression result
     * @param {number} minX - Minimum X-axis value from measured data
     * @param {number} maxX - Maximum X-axis value from measured data
     * @param {number} stepSize - Interval between points (default 1 degree)
     * @param {object|null} operatingLimits - {min, max} from config, or null for 10% extrapolation
     * @returns {object} - {interpolated: Array, extrapolated: Array, full: Array, extendedRange: {min, max}}
     */
    
    if (!regression) return {
        interpolated: [],
        extrapolated: [],
        full: [],
        extendedRange: { min: minX, max: maxX }
    };
    
    // Determine extended range based on operating limits or fallback to 10% extrapolation
    let extendedMinX, extendedMaxX;
    let usedOperatingLimits = false;
    
    if (operatingLimits && (operatingLimits.min !== 0 || operatingLimits.max !== 0)) {
        // Use operating limits from config
        extendedMinX = operatingLimits.min;
        extendedMaxX = operatingLimits.max;
        usedOperatingLimits = true;
    } else if (operatingLimits && operatingLimits.physicalMin !== undefined) {
        // Use physical limits if operating are 0 (meaning "use physical")
        extendedMinX = operatingLimits.physicalMin;
        extendedMaxX = operatingLimits.physicalMax;
        usedOperatingLimits = true;
    } else {
        // Fallback to 10% extrapolation
        const range = maxX - minX;
        const extension = range * 0.10;
        extendedMinX = minX - extension;
        extendedMaxX = maxX + extension;
    }
    
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
            usedOperatingLimits: usedOperatingLimits
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
            
            // Get operating limits from jointConfigData if available
            let operatingLimits = null;
            if (typeof jointConfigData !== 'undefined' && jointConfigData && jointConfigData.dofs) {
                const dofConfig = jointConfigData.dofs.find(d => d.index === dof);
                if (dofConfig) {
                    operatingLimits = {
                        min: dofConfig.operating_min || 0,
                        max: dofConfig.operating_max || 0,
                        physicalMin: dofConfig.min_angle,
                        physicalMax: dofConfig.max_angle
                    };
                    console.log(`DOF ${dof}: Using operating limits [${operatingLimits.min}, ${operatingLimits.max}] (physical: [${operatingLimits.physicalMin}, ${operatingLimits.physicalMax}])`);
                }
            }
            
            // Generate interpolated and extrapolated points with smart step
            const agonistExtended = generateInterpolatedAndExtrapolatedPoints(
                agonistRegression, minJointAngle, maxJointAngle, smartStepSize, operatingLimits
            );
            const antagonistExtended = generateInterpolatedAndExtrapolatedPoints(
                antagonistRegression, minJointAngle, maxJointAngle, smartStepSize, operatingLimits
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
                    used_operating_limits: agonistExtended.extendedRange.usedOperatingLimits,
                    operating_range: operatingLimits ? `[${operatingLimits.min || operatingLimits.physicalMin}, ${operatingLimits.max || operatingLimits.physicalMax}]` : 'N/A (10% extrapolation)',
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
            uses_operating_limits: true // Uses operating_min/max from config if available
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
    appendStatusMessage(`Exported complete file for ${jointName}: ${summary.total_dofs} DOF, ${summary.total_original_points} original points, ${summary.total_interpolated_points} interpolated, ${summary.total_extrapolated_points} extrapolated (operating range)`);
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
 * Checks if element is scrolled to bottom (with tolerance for padding)
 * @param {HTMLElement} element - Element to check
 * @returns {boolean} True if element is scrolled to bottom
 */
function isScrolledToBottom(element) {
    // Increased threshold to account for padding-bottom and ::after pseudo-element
    const threshold = 100; // Tolerance in pixels (covers 3rem padding + 2rem ::after)
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

// =============================================================================
// CAN INTERFACE MANAGEMENT
// =============================================================================

let availableCanInterfaces = [];
let selectedCanInterface = null;
let canConnectionState = {
    connected: false,
    last_status: {},
    stats: {}
};
let canStatusPollHandle = null;

function fetchCanInterfaces(options = {}) {
    const { showStatus = false } = options;
    return $.ajax({
        url: '/can_interfaces',
        method: 'GET',
        dataType: 'json'
    }).done(response => {
        if (response.status === 'success') {
            availableCanInterfaces = response.interfaces || [];
            updateCanInterfaceSelectUI();
            if (showStatus) {
                appendStatusMessage(`🔌 CAN interfaces detected: ${availableCanInterfaces.length}`);
            }
        } else {
            availableCanInterfaces = [];
            updateCanInterfaceSelectUI();
            if (showStatus) {
                appendStatusMessage(`⚠️ ${response.message || 'No CAN interfaces found'}`);
            }
        }
    }).fail((xhr, status, error) => {
        appendStatusMessage(`⚠️ Error fetching CAN interfaces: ${error}`);
        availableCanInterfaces = [];
        updateCanInterfaceSelectUI();
    });
}

function updateCanInterfaceSelectUI() {
    const select = $("#canInterfaceSelect");
    if (!select.length) {
        return;
    }

    const currentSelection = select.val();
    select.empty();
    select.append('<option value="">Select CAN interface</option>');

    availableCanInterfaces.forEach(iface => {
        const option = $('<option></option>')
            .val(iface.value)
            .text(iface.display_name);
        select.append(option);
    });

    // Restore previous selection if still available
    if (currentSelection) {
        select.val(currentSelection);
    }

    updateCanInterfaceHint();
}

function updateCanInterfaceHint() {
    const hint = $("#canInterfaceHint");
    if (!hint.length) {
        return;
    }

    const selectedValue = $("#canInterfaceSelect").val();
    
    if (!selectedValue) {
        hint.text('No CAN interface selected');
        hint.removeClass('text-green-600').addClass('text-gray-500');
        return;
    }

    try {
        const config = JSON.parse(selectedValue);
        hint.text(`Selected: ${config.interface} on ${config.channel}`);
        hint.removeClass('text-gray-500').addClass('text-green-600');
    } catch (e) {
        hint.text('Invalid interface configuration');
        hint.removeClass('text-green-600').addClass('text-red-600');
    }
}

function testCanInitialization() {
    const selectedValue = $("#canInterfaceSelect").val();
    
    if (!selectedValue) {
        appendStatusMessage("⚠️ Please select a CAN interface first");
        return;
    }

    const testBtn = $("#testCanInit");
    testBtn.prop('disabled', true);
    testBtn.html('<i class="fas fa-spinner fa-spin mr-1"></i>Testing...');
    
    appendStatusMessage("🔄 Testing CAN bus initialization...");

    $.ajax({
        url: '/can_test_init',
        method: 'POST',
        contentType: 'application/json',
        dataType: 'json',
        data: JSON.stringify({ config: selectedValue }),
        success: function(response) {
            if (response.status === 'success') {
                const info = response.bus_info;
                appendStatusMessage(`✅ CAN bus initialized successfully!`);
                appendStatusMessage(`   Interface: ${info.interface} on ${info.channel}`);
                appendStatusMessage(`   Bitrate: ${info.bitrate}`);
                
                if (info.test_message) {
                    appendStatusMessage(`   📨 Received message: ID=0x${info.test_message.arbitration_id}`);
                } else {
                    appendStatusMessage(`   ℹ️  No messages received (normal with no devices)`);
                }
            } else {
                appendStatusMessage(`❌ CAN test failed: ${response.message}`);
            }
        },
        error: function(xhr) {
            const errorMsg = xhr.responseJSON?.message || 'Unknown error';
            const errorType = xhr.responseJSON?.error_type || 'unknown';
            
            if (errorType === 'can_error') {
                appendStatusMessage(`❌ CAN error: ${errorMsg}`);
                appendStatusMessage(`   💡 Check: cable, termination, bitrate`);
            } else {
                appendStatusMessage(`❌ Test error: ${errorMsg}`);
            }
        },
        complete: function() {
            testBtn.prop('disabled', false);
            testBtn.html('<i class="fas fa-vial mr-1"></i>Test');
        }
    });
}

function connectCanInterface() {
    const selectedValue = $("#canInterfaceSelect").val();
    
    if (!selectedValue) {
        appendStatusMessage("⚠️ Select a CAN interface before connecting.");
        return;
    }

    const connectBtn = $("#connectCanBtn");
    connectBtn.prop("disabled", true).html('<i class="fas fa-spinner fa-spin mr-1"></i>Connecting...');

    $.ajax({
        url: '/can/connect',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ config: selectedValue })
    }).done(response => {
        if (response.status === 'success') {
            selectedCanInterface = selectedValue;
            appendStatusMessage(`✅ ${response.message || 'CAN interface connected'}`);
            fetchCanStatus({ showStatus: true });
        } else {
            appendStatusMessage(`❌ ${response.message || 'Failed to connect CAN interface'}`);
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ CAN connect error: ${message}`);
    }).always(() => {
        connectBtn.prop("disabled", false).html('<i class="fas fa-plug mr-1"></i>Connect');
    });
}

function disconnectCanInterface() {
    const disconnectBtn = $("#disconnectCanBtn");
    disconnectBtn.prop("disabled", true).html('<i class="fas fa-spinner fa-spin mr-1"></i>Disconnecting...');

    $.ajax({
        url: '/can/disconnect',
        method: 'POST'
    }).done(response => {
        if (response.status === 'success') {
            appendStatusMessage("🔌 CAN interface disconnected");
        } else {
            appendStatusMessage(`⚠️ ${response.message || 'Failed to disconnect CAN interface'}`);
        }
        fetchCanStatus();
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ CAN disconnect error: ${message}`);
    }).always(() => {
        disconnectBtn.prop("disabled", false).html('<i class="fas fa-unlink mr-1"></i>Disconnect');
    });
}

function fetchCanStatus(options = {}) {
    const { showStatus = false } = options;
    return $.ajax({
        url: '/can/status',
        method: 'GET',
        dataType: 'json'
    }).done(response => {
        if (response.status === 'success') {
            canConnectionState = response.state || {};
            updateCanStatusUI(canConnectionState);
            if (showStatus) {
                appendStatusMessage(`ℹ️ CAN status: ${canConnectionState.connected ? 'connected' : 'disconnected'}`);
            }
        } else if (showStatus) {
            appendStatusMessage(`⚠️ ${response.message || 'CAN status unavailable'}`);
        }
    }).fail(xhr => {
        if (xhr.status === 503) {
            updateCanStatusUI({ connected: false, error: 'CAN not available on this system' });
            if (showStatus) {
                appendStatusMessage('⚠️ CAN subsystem not available');
            }
        } else if (showStatus) {
            appendStatusMessage(`⚠️ Error fetching CAN status: ${xhr.statusText}`);
        }
    });
}

function updateCanStatusUI(state = {}) {
    const badge = $("#canConnectionBadge");
    const details = $("#canConnectionDetails");
    const statusList = $("#canStatusList");
    const rawMessages = $("#canRawMessages");

    const connected = !!state.connected;
    if (badge.length) {
        const baseClasses = "inline-flex items-center rounded-full px-2 py-0.5 text-xs font-semibold";
        if (connected) {
            badge.attr("class", `${baseClasses} bg-green-100 text-green-800`);
            badge.text("Connected");
        } else {
            badge.attr("class", `${baseClasses} bg-gray-200 text-gray-700`);
            badge.text("Disconnected");
        }
    }

    if (details.length) {
        if (connected && state.config) {
            const cfg = state.config;
            const bitrate = cfg.bitrate ? `${(cfg.bitrate / 1000).toFixed(0)} kbps` : 'unknown bitrate';
            details.text(`${cfg.interface || '??'} @ ${cfg.channel || 'N/A'} (${bitrate})`);
        } else if (state.error) {
            details.text(state.error);
        } else {
            details.text('Select interface and press Connect');
        }
    }

    if (statusList.length) {
        statusList.empty();
        if (state.last_status) {
            Object.values(state.last_status).forEach(entry => {
                const item = $('<div class="flex justify-between text-xs font-mono"></div>');
                const joint = $('<span></span>').text(`${entry.joint} · DOF${entry.dof_index}`);
                const value = $('<span></span>').text(`${entry.current_angle_deg.toFixed(2)}° → ${entry.target_angle_deg.toFixed(2)}° (${entry.progress_pct}% )`);
                item.append(joint).append(value);
                statusList.append(item);
            });
        } else {
            statusList.append('<div class="text-xs text-gray-500">No status frames received yet</div>');
        }
    }

    if (rawMessages.length) {
        rawMessages.empty();
        if (state.status_messages && state.status_messages.length > 0) {
            state.status_messages.slice(0, 6).forEach(msg => {
                const text = msg.type === "status"
                    ? `${msg.data.arbitration_id} joint=${msg.data.joint} flags=0x${msg.data.flags.toString(16)}`
                    : `${msg.frame.id} data=${msg.frame.data}`;
                rawMessages.append(`<div class="text-xs font-mono">${text}</div>`);
            });
        } else {
            rawMessages.append('<div class="text-xs text-gray-500">No frames logged</div>');
        }
    }
}

function sendCanTimeSyncCommand() {
    $.ajax({
        url: '/can/time_sync',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({})
    }).done(response => {
        if (response.status === 'success') {
            appendStatusMessage(`🕒 Time sync @ ${response.result?.timestamp_ms || 'unknown'} ms`);
        } else {
            appendStatusMessage(`⚠️ ${response.message || 'Time sync failed'}`);
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Time sync error: ${message}`);
    });
}

function sendCanEmergencyStop() {
    $.ajax({
        url: '/can/emergency_stop',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ reason_code: 1 })
    }).done(response => {
        if (response.status === 'success') {
            appendStatusMessage('🛑 Emergency stop broadcast via CAN');
        } else {
            appendStatusMessage(`⚠️ ${response.message || 'Emergency stop failed'}`);
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Emergency stop error: ${message}`);
    });
}

/**
 * Update CAN Motion Control panel to reflect selected joint
 * Called when jointSelect changes
 */
function updateCanMotionJoint() {
    const joint = $("#jointSelect").val();
    
    // Update the joint label in CAN Motion Control panel
    const label = $("#canMotionJointLabel");
    if (label.length) {
        label.text(joint);
    }
    
    // Update DOF options based on selected joint
    updateCanWaypointDofOptions();
}

function updateCanWaypointDofOptions() {
    const joint = $("#jointSelect").val();
    const dofSelect = $("#canWaypointDof");
    if (!joint || !dofSelect.length || !jointConfigData || !jointConfigData.joints) {
        return;
    }

    const configKey = joint.toLowerCase();
    const jointEntry = jointConfigData.joints[configKey];
    if (!jointEntry) {
        dofSelect.empty().append('<option value="0">DOF 0</option>');
        return;
    }

    dofSelect.empty();
    jointEntry.dofs.forEach((dof, idx) => {
        const label = dof.name ? dof.name.replace('_', ' ') : `DOF ${idx}`;
        dofSelect.append(`<option value="${idx}">${idx} · ${label}</option>`);
    });
}

function sendCanWaypointCommand() {
    const joint = $("#jointSelect").val();
    const dofIndex = parseInt($("#canWaypointDof").val(), 10) || 0;
    const angle = parseFloat($("#canWaypointAngle").val());
    const arrivalOffset = parseInt($("#canWaypointArrival").val(), 10) || 50;

    if (!joint) {
        appendStatusMessage("⚠️ Select a joint in Joint & Connection Setup.");
        return;
    }
    if (Number.isNaN(angle)) {
        appendStatusMessage("⚠️ Enter a valid angle in degrees.");
        return;
    }

    // Set interpolation mode before sending waypoint
    const interpolationMode = $('#singleWaypointInterpolation').val() || 'linear';
    $.ajax({
        url: '/can/interpolation_mode',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ mode: interpolationMode }),
        async: false  // Ensure mode is set before waypoint
    });
    const modeLabel = interpolationMode === 'cosine' ? 'SMOOTH' : 'LINEAR';

    // Build Multi-DOF format: set only the target DOF, null for others
    const angles = [null, null, null];
    angles[dofIndex] = angle;

    $.ajax({
        url: '/can/waypoint',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({
            joint: joint,
            angles_deg: angles,
            t_offset_ms: arrivalOffset
        })
    }).done(response => {
        if (response.status === 'success') {
            appendStatusMessage(`📡 Waypoint sent: ${joint} DOF${dofIndex} @ ${angle}° [${modeLabel}]`);
        } else {
            appendStatusMessage(`⚠️ ${response.message || 'Failed to send waypoint'}`);
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Waypoint error: ${message}`);
    });
}

/**
 * Send multiple waypoints that describe a COSINE curve (for comparison with single SMOOTH waypoint)
 * This generates N waypoints following the COSINE S-curve formula, then sends them with LINEAR interpolation.
 * The result SHOULD be identical to a single waypoint with COSINE interpolation.
 */
function sendMultiWaypointSmoothCurve() {
    const joint = $("#jointSelect").val();
    const dofIndex = parseInt($("#canWaypointDof").val(), 10) || 0;
    const targetAngle = parseFloat($("#canWaypointAngle").val());
    const totalTimeMs = parseInt($("#canWaypointArrival").val(), 10) || 500;
    
    // Slider now represents points per second (rate)
    const waypointRate = parseInt($("#multiWpPoints").val(), 10) || 100;
    // Calculate number of points from rate: numPoints = rate × (totalTime / 1000)
    const numPoints = Math.max(2, Math.round(waypointRate * (totalTimeMs / 1000)));
    const deltaT = Math.round(1000 / waypointRate);  // Δt = 1000ms / rate

    if (!joint) {
        appendStatusMessage("⚠️ Select a joint in Joint & Connection Setup.");
        return;
    }
    if (Number.isNaN(targetAngle)) {
        appendStatusMessage("⚠️ Enter a valid angle in degrees.");
        return;
    }

    // Get current angle from encoder display (if available)
    // Extract joint type from joint name (e.g., "KNEE_RIGHT" -> "knee")
    const jointType = joint.split('_')[0].toLowerCase();
    let startAngle = 0;
    
    // Try joint-specific encoder display (e.g., #kneeEncoderDof0)
    const jointEncoderText = $(`#${jointType}EncoderDof${dofIndex}`).text();
    if (jointEncoderText && jointEncoderText !== '-') {
        startAngle = parseFloat(jointEncoderText.replace('°', '')) || 0;
    } else {
        // Fallback to generic encoder display
        const encoderText = $(`#encoder${dofIndex}Value`).text();
        if (encoderText && encoderText !== '-') {
            startAngle = parseFloat(encoderText) || 0;
        }
    }

    appendStatusMessage(`🔬 Multi-WP: ${startAngle.toFixed(1)}° → ${targetAngle}° @ ${waypointRate} pts/s (${numPoints} pts, Δt=${deltaT}ms)`);

    // Force LINEAR interpolation (the COSINE curve is in the waypoints themselves)
    $.ajax({
        url: '/can/interpolation_mode',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ mode: 'linear' }),
        async: false
    });

    // Generate waypoints following COSINE S-curve
    const waypoints = [];
    const actualDeltaT = totalTimeMs / numPoints;
    
    // Estimate batch send time for initial offset calculation
    // Backend will compensate for actual elapsed time, so this is just a buffer
    const estimatedBatchSendTime = numPoints * 3;  // ~3ms per waypoint
    const initialOffset = estimatedBatchSendTime + 300; // Buffer before first waypoint arrives

    for (let i = 0; i <= numPoints; i++) {
        const t = i / numPoints; // 0.0 to 1.0
        
        // COSINE interpolation formula (same as firmware)
        const smoothT = 0.5 * (1 - Math.cos(t * Math.PI));
        
        // Calculate interpolated angle
        const angle = startAngle + (targetAngle - startAngle) * smoothT;
        
        // Calculate desired arrival time from batch start
        // Backend compensates for actual elapsed time when sending each waypoint
        const desiredArrivalFromStart = initialOffset + (i * actualDeltaT);
        
        // Build waypoint
        const angles = [null, null, null];
        angles[dofIndex] = angle;
        
        waypoints.push({
            joint: joint,
            angles_deg: angles,
            t_offset_ms: Math.round(desiredArrivalFromStart)
        });
    }

    appendStatusMessage(`📊 Generated ${waypoints.length} waypoints`);

    // Send as batch
    $.ajax({
        url: '/can/waypoint_batch',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ joint: joint, waypoints: waypoints })
    }).done(response => {
        if (response.status === 'success') {
            appendStatusMessage(`✅ Multi-WP batch sent: ${waypoints.length} waypoints [LINEAR interp]`);
        } else {
            appendStatusMessage(`⚠️ ${response.message || 'Failed to send batch'}`);
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Batch error: ${message}`);
    });
}

/**
 * Update sinusoid statistics display when parameters change
 * @param {number} rate - Points per second (optional, reads from UI if not provided)
 */
function updateSinusoidStats(rate) {
    // Input is now points per second (rate)
    const pointsPerSecond = parseInt(rate, 10) || parseInt($("#waypointDensity").val(), 10) || 100;
    const cycleDurationSeconds = parseFloat($("#sinusoidCycleDuration").val()) || 3;
    const numCycles = parseInt($("#sinusoidCycles").val(), 10) || 2;
    const totalDurationSeconds = cycleDurationSeconds * numCycles;
    
    // Calculate interval between points: Δt = 1000ms / rate
    const intervalMs = Math.round(1000 / pointsPerSecond);
    
    // Total waypoints for entire oscillation
    const totalWaypoints = Math.round(pointsPerSecond * totalDurationSeconds);
    
    // Calculate frequency in Hz (1 / cycle duration)
    const freqHz = (1 / cycleDurationSeconds).toFixed(2);
    
    // Update UI elements
    const updateElement = (id, value) => {
        const el = document.getElementById(id);
        if (el) el.textContent = value;
    };
    
    updateElement('waypointDensityValue', pointsPerSecond);  // Now shows pts/s
    updateElement('waypointIntervalValue', intervalMs);      // Δt in ms
    updateElement('totalWaypointsValue', totalWaypoints);    // Total waypoints
    updateElement('sinusoidCycleDurationDisplay', cycleDurationSeconds);
    updateElement('sinusoidCyclesDisplay', numCycles);
    updateElement('sinusoidTotalDuration', totalDurationSeconds);
    updateElement('sinusoidTotalDurationDisplay', totalDurationSeconds);
    updateElement('sinusoidFreqDisplay', freqHz);
    
    // Warn if buffer might overflow (2000 max)
    const bufferWarning = document.getElementById('bufferWarning');
    if (bufferWarning) {
        if (totalWaypoints > 2000) {
            bufferWarning.textContent = `⚠️ ${totalWaypoints} pts > 2000 buffer!`;
            bufferWarning.style.display = 'inline';
        } else {
            bufferWarning.style.display = 'none';
        }
    }
}

function sendCanWaypointSequence() {
    const joint = $("#jointSelect").val();
    const mode = parseInt($("#canWaypointMode").val(), 10) || 1;
    
    // Get parameters from UI
    // Slider now represents points per second (rate)
    const waypointRate = parseInt($("#waypointDensity").val(), 10) || 100;
    const cycleDurationSeconds = parseFloat($("#sinusoidCycleDuration").val()) || 3;
    const numCycles = parseInt($("#sinusoidCycles").val(), 10) || 2;
    const totalDurationSeconds = cycleDurationSeconds * numCycles;
    const totalDuration = totalDurationSeconds * 1000;  // Convert to ms
    const frequency = 1 / cycleDurationSeconds;  // Hz (frequency of a single cycle)
    
    // Calculate total waypoints from rate: pts/s × duration
    const totalWaypoints = Math.round(waypointRate * totalDurationSeconds);

    if (!joint) {
        appendStatusMessage("⚠️ Select a joint in Joint & Connection Setup.");
        return;
    }
    
    // Get active DOFs and their oscillation parameters
    const activeDofs = [];
    for (let dof = 0; dof < 3; dof++) {
        const isActive = $(`#sinusoidDof${dof}Active`).is(':checked');
        const container = $(`#sinusoidDof${dof}Container`);
        
        if (isActive && (dof === 0 || container.is(':visible'))) {
            const minAngle = parseFloat($(`#sinusoidDof${dof}Min`).val()) || -10;
            const maxAngle = parseFloat($(`#sinusoidDof${dof}Max`).val()) || 10;
            const centerAngle = (minAngle + maxAngle) / 2;
            const amplitude = (maxAngle - minAngle) / 2;
            
            activeDofs.push({
                index: dof,
                centerAngle: centerAngle,
                amplitude: amplitude,
                minAngle: minAngle,
                maxAngle: maxAngle
            });
        }
    }
    
    if (activeDofs.length === 0) {
        appendStatusMessage("⚠️ Select at least one DOF for the sinusoid test.");
        return;
    }

    // Disable button during sequence
    const btn = $("#sendCanWaypointSequenceBtn");
    btn.prop('disabled', true);
    btn.html('<i class="fas fa-spinner fa-spin mr-1"></i>Sending...');

    // Force LINEAR interpolation for sinusoidal trajectory
    // Sinusoid uses many waypoints that form the curve - LINEAR connects them without stopping
    $.ajax({
        url: '/can/interpolation_mode',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ mode: 'linear' }),
        async: false
    });
    appendStatusMessage(`⚙️ Interpolation: LINEAR (sinusoid)`);

    // Calculate timing parameters for batch sending
    // Estimate batch send time for initial offset calculation (~3ms per waypoint)
    // Backend compensates for actual elapsed time when sending each waypoint
    const estimatedBatchSendTime = totalWaypoints * 3;
    // Initial offset must be > batch send time + network latency + processing margin
    const initialOffset = Math.max(500, estimatedBatchSendTime + 300);
    
    // Generate waypoints for all active DOFs
    const testSequence = [];
    for (let i = 0; i < totalWaypoints; i++) {
        const t = (i / (totalWaypoints - 1)) * totalDuration;  // Time in ms
        const tSeconds = t / 1000;
        
        // Calculate angles for each active DOF
        const angles = [null, null, null];
        activeDofs.forEach(dof => {
            // Sinusoidal angle: center + amplitude * sin(2π * frequency * t)
            const angle = dof.centerAngle + dof.amplitude * Math.sin(2 * Math.PI * frequency * tSeconds);
            angles[dof.index] = Math.round(angle * 100) / 100;
        });
        
        testSequence.push({
            angles: angles,
            arrival_offset_ms: Math.round(t) + initialOffset
        });
    }
    
    // Add FINAL waypoint at center position (0° for symmetric oscillation)
    // This ensures the sequence ends exactly at center
    // With batch mode, order is guaranteed so we push to the end
    const finalAngles = [null, null, null];
    activeDofs.forEach(dof => {
        finalAngles[dof.index] = dof.centerAngle;  // Exactly at center
    });
    testSequence.push({
        angles: finalAngles,
        arrival_offset_ms: totalDuration + initialOffset + 200  // 200ms after last sinusoid point
    });
    
    // Calculate delta-t between points for logging (Δt = 1000 / rate)
    const deltaT = Math.round(1000 / waypointRate);
    
    // Log active DOFs info
    const dofInfo = activeDofs.map(d => `DOF${d.index}[${d.minAngle}°↔${d.maxAngle}°]`).join(', ');
    const centerInfo = activeDofs.map(d => `${d.centerAngle}°`).join(', ');
    appendStatusMessage(`🚀 Sending SINUSOIDAL sequence for ${joint}`);
    appendStatusMessage(`   📊 ${testSequence.length} waypoints @ ${waypointRate} pts/s (Δt=${deltaT}ms)`);
    appendStatusMessage(`   📈 ${numCycles} cycles × ${cycleDurationSeconds}s = ${totalDurationSeconds}s total @ ${frequency.toFixed(2)}Hz`);
    appendStatusMessage(`   🎯 Active: ${dofInfo}`);
    appendStatusMessage(`   🏁 Final target: ${centerInfo}`);

    // === DETERMINISTIC BATCH SENDING ===
    // Send all waypoints in a single request - server forwards them sequentially
    // This guarantees order and completeness (no lost waypoints)
    
    appendStatusMessage(`📡 Sending ${testSequence.length} waypoints (batch mode)...`);
    
    // Convert to batch format
    const batchPayload = testSequence.map(wp => ({
        angles_deg: wp.angles,
        t_offset_ms: wp.arrival_offset_ms
    }));
    
    $.ajax({
        url: '/can/waypoint_batch',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({
            joint: joint,
            waypoints: batchPayload
        })
    }).done(response => {
        if (response.status === 'success') {
            const result = response.result || {};
            appendStatusMessage(`📤 Batch sent: ${result.success}/${result.total} waypoints queued`);
            
            if (result.errors > 0) {
                appendStatusMessage(`  ⚠️ ${result.errors} waypoints failed`);
            }
            
            // Wait for sequence to complete
            const waitTime = totalDuration + initialOffset + 500;
            setTimeout(() => {
                btn.prop('disabled', false);
                btn.html('<i class="fas fa-wave-square mr-1"></i>Send Sinusoid');
                appendStatusMessage(`✅ Sequence execution complete`);
            }, waitTime);
        } else {
            appendStatusMessage(`❌ Batch failed: ${response.message}`);
            btn.prop('disabled', false);
            btn.html('<i class="fas fa-wave-square mr-1"></i>Send Sinusoid');
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Batch error: ${message}`);
        btn.prop('disabled', false);
        btn.html('<i class="fas fa-wave-square mr-1"></i>Send Sinusoid');
    });
}

/**
 * Send oscillation using COSINE S-curve waypoints with LINEAR interpolation.
 * Like Multi-WP: waypoints themselves describe a smooth S-curve, firmware uses LINEAR between them.
 * Oscillates directly between min and max (no center point).
 */
function sendCosineOscillation() {
    const joint = $("#jointSelect").val();
    
    // Get parameters from UI (same slider as sinusoid)
    // Slider now represents points per second (rate)
    const waypointRate = parseInt($("#waypointDensity").val(), 10) || 100;
    const cycleDurationSeconds = parseFloat($("#sinusoidCycleDuration").val()) || 3;
    const numCycles = parseInt($("#sinusoidCycles").val(), 10) || 2;
    const totalDurationSeconds = cycleDurationSeconds * numCycles;
    const totalDuration = totalDurationSeconds * 1000;  // Convert to ms

    if (!joint) {
        appendStatusMessage("⚠️ Select a joint in Joint & Connection Setup.");
        return;
    }
    
    // Get active DOFs and their oscillation parameters
    const activeDofs = [];
    for (let dof = 0; dof < 3; dof++) {
        const isActive = $(`#sinusoidDof${dof}Active`).is(':checked');
        const container = $(`#sinusoidDof${dof}Container`);
        
        if (isActive && (dof === 0 || container.is(':visible'))) {
            const minAngle = parseFloat($(`#sinusoidDof${dof}Min`).val()) || -10;
            const maxAngle = parseFloat($(`#sinusoidDof${dof}Max`).val()) || 10;
            
            activeDofs.push({
                index: dof,
                minAngle: minAngle,
                maxAngle: maxAngle
            });
        }
    }
    
    if (activeDofs.length === 0) {
        appendStatusMessage("⚠️ Select at least one DOF for the oscillation test.");
        return;
    }

    // Disable button during sequence
    const btn = $("#sendCosineOscillationBtn");
    btn.prop('disabled', true);
    btn.html('<i class="fas fa-spinner fa-spin mr-1"></i>Sending...');

    // Force LINEAR interpolation - the S-curve is in the waypoints themselves
    $.ajax({
        url: '/can/interpolation_mode',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ mode: 'linear' }),
        async: false
    });
    appendStatusMessage(`⚙️ Interpolation: LINEAR (S-curve in waypoints)`);

    // Calculate timing parameters (same approach as Multi-WP)
    // waypointRate is points per second, convert to points per half-cycle
    const halfCycleSeconds = cycleDurationSeconds / 2;
    const halfCycleMs = halfCycleSeconds * 1000;
    const totalHalfCycles = numCycles * 2;
    const pointsPerHalfCycle = Math.round(waypointRate * halfCycleSeconds);
    const deltaT = 1000 / waypointRate;  // Time between points = 1000ms / rate
    
    // Estimate total waypoints for batch timing
    const estimatedTotalWaypoints = Math.round(waypointRate * totalDurationSeconds);
    const estimatedBatchSendTime = estimatedTotalWaypoints * 3;  // ~3ms per waypoint
    const initialOffset = Math.max(500, estimatedBatchSendTime + 300);
    
    // Warn if exceeding buffer (2000 waypoints max)
    if (estimatedTotalWaypoints > 2000) {
        appendStatusMessage(`⚠️ Warning: ${estimatedTotalWaypoints} waypoints exceeds buffer (2000). Reduce rate or cycles.`);
    }
    
    // Generate waypoints using same approach as Multi-WP
    // Each half-cycle (min→max or max→min) follows a COSINE S-curve
    const testSequence = [];
    
    for (let halfCycle = 0; halfCycle < totalHalfCycles; halfCycle++) {
        const isGoingToMax = (halfCycle % 2 === 0);
        
        // Generate points for this half-cycle (same as Multi-WP: 0 to pointsPerHalfCycle inclusive)
        const startIndex = (halfCycle === 0) ? 0 : 1;  // Skip only first point of subsequent half-cycles
        
        for (let p = startIndex; p <= pointsPerHalfCycle; p++) {
            // Progress within this half-cycle (0.0 to 1.0) - SAME AS MULTI-WP
            const t = p / pointsPerHalfCycle;
            
            // Apply COSINE S-curve: smooth start and end (IDENTICAL TO MULTI-WP)
            const smoothT = 0.5 * (1 - Math.cos(t * Math.PI));
            
            // Calculate angles for each DOF
            const angles = [null, null, null];
            activeDofs.forEach(dof => {
                const fromAngle = isGoingToMax ? dof.minAngle : dof.maxAngle;
                const toAngle = isGoingToMax ? dof.maxAngle : dof.minAngle;
                const angle = fromAngle + (toAngle - fromAngle) * smoothT;
                angles[dof.index] = Math.round(angle * 100) / 100;
            });
            
            // Calculate desired arrival time from batch start
            // Backend compensates for actual elapsed time when sending each waypoint
            const desiredArrivalFromStart = initialOffset + (halfCycle * halfCycleMs) + (p * deltaT);
            
            testSequence.push({
                angles: angles,
                arrival_offset_ms: Math.round(desiredArrivalFromStart)
            });
        }
    }
    
    // Calculate final time based on last waypoint
    const finalTime = testSequence.length > 0 
        ? testSequence[testSequence.length - 1].arrival_offset_ms 
        : initialOffset + totalDuration;
    
    // Log info (deltaT already declared above)
    const dofInfo = activeDofs.map(d => `DOF${d.index}[${d.minAngle}°↔${d.maxAngle}°]`).join(', ');
    appendStatusMessage(`🚀 Sending S-curve oscillation for ${joint}`);
    appendStatusMessage(`   📊 ${testSequence.length} waypoints @ ${waypointRate} pts/s (Δt=${Math.round(deltaT)}ms)`);
    appendStatusMessage(`   📈 ${numCycles} cycles × ${cycleDurationSeconds}s = ${totalDurationSeconds}s total`);
    appendStatusMessage(`   🎯 Active: ${dofInfo} (min↔max direct)`);

    // Convert to batch format
    const batchPayload = testSequence.map(wp => ({
        angles_deg: wp.angles,
        t_offset_ms: wp.arrival_offset_ms
    }));
    
    $.ajax({
        url: '/can/waypoint_batch',
        method: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({
            joint: joint,
            waypoints: batchPayload
        })
    }).done(response => {
        if (response.status === 'success') {
            const result = response.result || {};
            appendStatusMessage(`📤 Batch sent: ${result.success}/${result.total} waypoints queued`);
            
            // Wait for sequence to complete
            const waitTime = finalTime + 500;
            setTimeout(() => {
                btn.prop('disabled', false);
                btn.html('<i class="fas fa-bezier-curve mr-1"></i>Oscillation (S-curve)');
                appendStatusMessage(`✅ S-curve oscillation complete`);
            }, waitTime);
        } else {
            appendStatusMessage(`❌ Batch failed: ${response.message}`);
            btn.prop('disabled', false);
            btn.html('<i class="fas fa-bezier-curve mr-1"></i>Oscillation (S-curve)');
        }
    }).fail(xhr => {
        const message = xhr.responseJSON?.message || xhr.statusText || 'Unknown error';
        appendStatusMessage(`❌ Batch error: ${message}`);
        btn.prop('disabled', false);
        btn.html('<i class="fas fa-bezier-curve mr-1"></i>Oscillation (S-curve)');
    });
}


function startCanStatusPolling() {
    if (canStatusPollHandle) {
        clearInterval(canStatusPollHandle);
    }
    fetchCanStatus();
    canStatusPollHandle = setInterval(fetchCanStatus, 3000);
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

// --- Loop Frequency Control Functions ---

/**
 * Update the frequency display labels when input values change
 */
function updateLoopFrequencyDisplays() {
    const innerPeriodUs = parseInt($('#innerLoopPeriod').val()) || 2000;
    const outerDivisor = parseInt($('#outerLoopDivisor').val()) || 5;
    
    const innerFreqHz = 1000000.0 / innerPeriodUs;
    const outerFreqHz = innerFreqHz / outerDivisor;
    
    $('#innerLoopFreqDisplay').text(`= ${innerFreqHz.toFixed(1)} Hz`);
    $('#outerLoopFreqDisplay').text(`= ${outerFreqHz.toFixed(1)} Hz`);
}

/**
 * Send loop frequency configuration to the firmware via CAN
 */
function updateLoopFrequencies() {
    const innerPeriodUs = parseInt($('#innerLoopPeriod').val()) || 2000;
    const outerDivisor = parseInt($('#outerLoopDivisor').val()) || 5;
    
    // Validate ranges
    if (innerPeriodUs < 500 || innerPeriodUs > 10000) {
        appendStatusMessage(`❌ Inner loop period must be between 500µs and 10000µs`, 'error');
        return;
    }
    if (outerDivisor < 1 || outerDivisor > 20) {
        appendStatusMessage(`❌ Outer loop divisor must be between 1 and 20`, 'error');
        return;
    }
    
    const innerFreqHz = 1000000.0 / innerPeriodUs;
    const outerFreqHz = innerFreqHz / outerDivisor;
    
    $.ajax({
        url: '/can/loop_frequencies',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({
            inner_period_us: innerPeriodUs,
            outer_divisor: outerDivisor
        }),
        success: function(response) {
            if (response.status === 'success') {
                appendStatusMessage(`✅ Loop frequencies updated: Inner=${innerFreqHz.toFixed(1)}Hz (${innerPeriodUs}µs), Outer=${outerFreqHz.toFixed(1)}Hz (÷${outerDivisor})`);
            } else {
                appendStatusMessage(`❌ Failed to update loop frequencies: ${response.message}`, 'error');
            }
        },
        error: function(xhr) {
            const errorMsg = xhr.responseJSON?.message || xhr.statusText;
            appendStatusMessage(`❌ Error updating loop frequencies: ${errorMsg}`, 'error');
        }
    });
}

// Attach event listeners for real-time frequency display updates
$(document).ready(function() {
    $('#innerLoopPeriod, #outerLoopDivisor').on('input', updateLoopFrequencyDisplays);
    // Initialize displays
    updateLoopFrequencyDisplays();
});

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
 * If sequence mode is active, adds step to sequence instead
 */
function setMultiDofQuickAngles(angle0, angle1) {
    $("#multiDofAngle0").val(angle0);
    $("#multiDofAngle1").val(angle1);
    updateMultiDofCommandPreview();
    
    // Check which mode is active
    if ($("#sequenceModeToggle").is(":checked")) {
        // Add to sequence instead of executing
        addStepToSequence(angle0, angle1);
        appendStatusMessage(`➕ Added to sequence: DOF0=${angle0}°, DOF1=${angle1}°`);
    } else if ($("#autoExecuteToggle").is(":checked")) {
        // Execute immediately (existing behavior)
        sendMultiDofMove();
        appendStatusMessage(`🚀 Auto-execute active: movement started immediately`);
    } else {
        // Normal mode: just set angles
        appendStatusMessage(`Quick angles set: DOF0=${angle0}°, DOF1=${angle1}°`);
    }
}

/**
 * Updates Multi-DOF command preview while user modifies parameters
 * Note: Preview textarea was removed from UI (was debug only)
 * Function kept for backwards compatibility but does nothing
 */
function updateMultiDofCommandPreview() {
    // Preview element removed from UI - this function is now a no-op
    // Kept to avoid breaking existing calls throughout the codebase
    return;
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
    
    // Determine joint type
    const joint = $("#jointSelect").val();
    const jointType = joint.split('_')[0]; // Extract KNEE, ANKLE or HIP
    
    // Informative header
    container.append(`
        <h4 class="text-sm font-medium text-gray-700 mb-2">
            <i class="fas fa-brain mr-1"></i>Smart Positions (mapping-based):
            ${isAutoExecuteEnabled ? 
                '<span class="text-orange-500 ml-2"><i class="fas fa-bolt"></i> Auto-execute active</span>' : 
                ''}
        </h4>
    `);
    
    // Generate smart combinations
    const dofs = Object.keys(ranges).map(Number).sort();
    const combinations = generateSmartCombinations(ranges, dofs);
    
    // Separate combinations by type
    const dof0Only = combinations.filter(c => c.angle0 !== 0 && c.angle1 === 0);
    const dof1Only = combinations.filter(c => c.angle0 === 0 && c.angle1 !== 0);
    const multiDof = combinations.filter(c => c.angle0 !== 0 && c.angle1 !== 0);
    
    // For KNEE: simplified single-DOF layout (no grid needed)
    if (jointType === 'KNEE' && dof0Only.length > 0) {
        container.append('<h5 class="text-xs font-medium text-gray-600 mt-3 mb-1">Positions:</h5>');
        const dof0Container = $('<div class="flex flex-wrap gap-2"></div>');
        
        // Zero button first
        dof0Container.append(`
            <button onclick="setMultiDofQuickAngles(0, 0)" 
                    class="bg-gray-500 hover:bg-gray-600 text-white text-xs py-2 rounded"
                    style="min-width: 70px; width: 70px;"
                    title="DOF 0: 0°">
                Zero
            </button>
        `);
        
        dof0Only.forEach(combo => {
            const { angle0, angle1, label, type } = combo;
            const colorClass = getButtonColorClass(type);
            
            dof0Container.append(`
                <button onclick="setMultiDofQuickAngles(${angle0}, ${angle1})" 
                        class="${colorClass} text-white text-xs py-2 rounded transition-colors"
                        style="min-width: 70px; width: 70px;"
                        title="DOF 0: ${angle0}°">
                    ${angle0}°
                </button>
            `);
        });
        
        container.append(dof0Container);
    }
    
    // Multi-DOF grid - Hide for KNEE, show for ANKLE/HIP
    if (jointType !== 'KNEE') {
        container.append('<h5 class="text-xs font-medium text-gray-600 mt-3 mb-1">Joint workspace - Top view:</h5>');
        
        // Generate 2D grid representing joint workspace
        // Rows: DOF 0 from +max to -min (top to bottom)
        // Cols: DOF 1 from -max to +max (left to right)
        
        const range0 = ranges[0];
        const range1 = ranges[1];
        
        const dof0Values = [];
        const dof1Values = [];
        
        // Generate DOF 0 values (rows): from max to min, step 5°
        for (let angle = Math.ceil(range0.extended_max / 5) * 5; angle >= Math.floor(range0.extended_min / 5) * 5; angle -= 5) {
            dof0Values.push(angle);
        }
        
        // Generate DOF 1 values (cols): from min to max, step 5°
        for (let angle = Math.floor(range1.extended_min / 5) * 5; angle <= Math.ceil(range1.extended_max / 5) * 5; angle += 5) {
            dof1Values.push(angle);
        }
        
        // Create grid container (centered)
        const gridCols = dof1Values.length;
        const multiDofContainer = $(`<div class="grid gap-0.5" style="grid-template-columns: repeat(${gridCols}, 36px); justify-content: center;"></div>`);
        
        // Generate grid buttons
        dof0Values.forEach(angle0 => {
            dof1Values.forEach(angle1 => {
                // Determine button color to create a cross pattern for single-DOF movements
                let colorClass;
                
                // Zero position (center of cross)
                if (angle0 === 0 && angle1 === 0) {
                    colorClass = 'bg-gray-500 hover:bg-gray-600';
                }
                // Horizontal axis: DOF 0 movements (DOF 1 = 0) - Blue like DOF 0 buttons
                else if (angle1 === 0) {
                    colorClass = 'bg-indigo-400 hover:bg-indigo-500';
                }
                // Vertical axis: DOF 1 movements (DOF 0 = 0) - Teal like DOF 1 buttons
                else if (angle0 === 0) {
                    colorClass = 'bg-teal-400 hover:bg-teal-500';
                }
                // Edge positions (multi-DOF extremes)
                else if (angle0 === dof0Values[0] || angle0 === dof0Values[dof0Values.length - 1] ||
                         angle1 === dof1Values[0] || angle1 === dof1Values[dof1Values.length - 1]) {
                    colorClass = 'bg-purple-500 hover:bg-purple-600';
                }
                // Internal multi-DOF positions
                else {
                    colorClass = 'bg-purple-400 hover:bg-purple-500';
                }
                
                multiDofContainer.append(`
                    <button onclick="setMultiDofQuickAngles(${angle0}, ${angle1})" 
                            class="${colorClass} text-white rounded transition-colors"
                            style="width: 36px; height: 36px; font-size: 7px; padding: 1px; line-height: 1.1;"
                            title="DOF 0: ${angle0}°, DOF 1: ${angle1}°${isAutoExecuteEnabled ? ' (auto-exec)' : ''}">
                        ${angle0}°<br>${angle1}°
                    </button>
                `);
            });
        });
        
        container.append(multiDofContainer);
    }
    
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
        
        // Generate significant points for this DOF (more granular)
        const significantAngles = [
            minAngle,                                  // 0%
            minAngle + (maxAngle - minAngle) * 0.125, // 12.5%
            minAngle + (maxAngle - minAngle) * 0.25,  // 25%
            minAngle + (maxAngle - minAngle) * 0.375, // 37.5%
            midAngle,                                  // 50%
            minAngle + (maxAngle - minAngle) * 0.625, // 62.5%
            minAngle + (maxAngle - minAngle) * 0.75,  // 75%
            minAngle + (maxAngle - minAngle) * 0.875, // 87.5%
            maxAngle                                   // 100%
        ].map(angle => Math.round(angle * 10) / 10); // Arrotonda a 1 decimale
        
        // Crea combinazioni per singolo DOF
        significantAngles.forEach((angle, index) => {
            const labels = ['Min', '12%', '25%', '37%', 'Mid', '62%', '75%', '87%', 'Max'];
            const types = ['min', 'eighth', 'quarter', 'threeeighth', 'mid', 'fiveeighth', 'threequarter', 'seveneighth', 'max'];
            
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
    
    // If both DOFs are present, generate coordinated combinations
    if (dofs.includes(0) && dofs.includes(1)) {
        const range0 = ranges[0];
        const range1 = ranges[1];
        
        // Interesting coordinated combinations
        const mid0 = (range0.extended_min + range0.extended_max) / 2;
        const mid1 = (range1.extended_min + range1.extended_max) / 2;
        
        const coordCombinations = [
            // Corner combinations (extremes)
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
            },
            
            // Center combination
            {
                angle0: mid0,
                angle1: mid1,
                label: 'Mid/Mid',
                type: 'coord-mid'
            },
            
            // Mid combinations with extremes (useful for testing transitions)
            {
                angle0: mid0,
                angle1: range1.extended_min,
                label: 'Mid/Min',
                type: 'coord-mid-edge'
            },
            {
                angle0: mid0,
                angle1: range1.extended_max,
                label: 'Mid/Max',
                type: 'coord-mid-edge'
            },
            {
                angle0: range0.extended_min,
                angle1: mid1,
                label: 'Min/Mid',
                type: 'coord-mid-edge'
            },
            {
                angle0: range0.extended_max,
                angle1: mid1,
                label: 'Max/Mid',
                type: 'coord-mid-edge'
            },
            
            // Additional intermediate points (25% and 75%)
            {
                angle0: range0.extended_min + (range0.extended_max - range0.extended_min) * 0.25,
                angle1: range1.extended_min + (range1.extended_max - range1.extended_min) * 0.75,
                label: '25%/75%',
                type: 'coord-intermediate'
            },
            {
                angle0: range0.extended_min + (range0.extended_max - range0.extended_min) * 0.75,
                angle1: range1.extended_min + (range1.extended_max - range1.extended_min) * 0.25,
                label: '75%/25%',
                type: 'coord-intermediate'
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
 * Returns the CSS class for the button color based on the type
 */
function getButtonColorClass(type) {
    const colorMap = {
        'coord-mid': 'bg-indigo-500 hover:bg-indigo-600',
        'coord-min': 'bg-purple-500 hover:bg-purple-600',
        'coord-max': 'bg-purple-500 hover:bg-purple-600',
        'coord-mixed': 'bg-violet-500 hover:bg-violet-600',
        'coord-mid-edge': 'bg-purple-400 hover:bg-purple-500',
        'coord-intermediate': 'bg-violet-400 hover:bg-violet-500',
        'mid': 'bg-indigo-400 hover:bg-indigo-500',
        'quarter': 'bg-blue-400 hover:bg-blue-500',
        'threequarter': 'bg-blue-400 hover:bg-blue-500',
        'eighth': 'bg-blue-300 hover:bg-blue-400',
        'threeeighth': 'bg-blue-300 hover:bg-blue-400',
        'fiveeighth': 'bg-blue-300 hover:bg-blue-400',
        'seveneighth': 'bg-blue-300 hover:bg-blue-400',
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
            <i class="fas fa-cog mr-1"></i>Default Positions (${jointType}):
            ${isAutoExecuteEnabled ? 
                '<span class="text-orange-500 ml-2"><i class="fas fa-bolt"></i> Auto-execute active</span>' : 
                ''}
        </h4>
    `);
    
    // Default buttons based on joint type
    const defaultButtons = getDefaultButtonsForJoint(jointType);
    
    // Check if buttons have section property (for ANKLE with organized layout)
    const hasSections = defaultButtons.some(btn => btn.section);
    
    if (hasSections && jointType === 'ANKLE') {
        // For ANKLE: generate 2D grid with cross-pattern highlighting for single-DOF
        const multiButtons = defaultButtons.filter(btn => btn.section === 'multi');
        
        if (multiButtons.length > 0) {
            container.append('<h5 class="text-xs font-medium text-gray-600 mt-3 mb-1">Joint workspace - Top view:</h5>');
            
            // For ANKLE: fixed grid from default positions
            // DOF 0: +25° to -50° (rows, top to bottom)
            // DOF 1: -25° to +25° (columns, left to right)
            
            const dof0Values = [];
            const dof1Values = [];
            
            // Generate DOF 0 values (rows): from +25 to -50, step -5
            for (let angle = 25; angle >= -50; angle -= 5) {
                dof0Values.push(angle);
            }
            
            // Generate DOF 1 values (cols): from -25 to +25, step +5
            for (let angle = -25; angle <= 25; angle += 5) {
                dof1Values.push(angle);
            }
            
            // Create grid container (centered)
            const gridCols = dof1Values.length;
            const multiContainer = $(`<div class="grid gap-0.5" style="grid-template-columns: repeat(${gridCols}, 36px); justify-content: center;"></div>`);
            
            // Generate grid buttons
            dof0Values.forEach(angle0 => {
                dof1Values.forEach(angle1 => {
                    // Determine button color to create a cross pattern for single-DOF movements
                    let colorClass;
                    
                    // Zero position (center of cross)
                    if (angle0 === 0 && angle1 === 0) {
                        colorClass = 'bg-gray-500 hover:bg-gray-600';
                    }
                    // Horizontal axis: DOF 0 movements (DOF 1 = 0) - Blue like DOF 0 buttons
                    else if (angle1 === 0) {
                        colorClass = 'bg-indigo-400 hover:bg-indigo-500';
                    }
                    // Vertical axis: DOF 1 movements (DOF 0 = 0) - Teal like DOF 1 buttons
                    else if (angle0 === 0) {
                        colorClass = 'bg-teal-400 hover:bg-teal-500';
                    }
                    // Edge positions (multi-DOF extremes)
                    else if (angle0 === 25 || angle0 === -50 || angle1 === -25 || angle1 === 25) {
                        colorClass = 'bg-purple-500 hover:bg-purple-600';
                    }
                    // Internal multi-DOF positions
                    else {
                        colorClass = 'bg-purple-400 hover:bg-purple-500';
                    }
                    
                    multiContainer.append(`
                        <button onclick="setMultiDofQuickAngles(${angle0}, ${angle1})" 
                                class="${colorClass} text-white rounded transition-colors"
                                style="width: 36px; height: 36px; font-size: 7px; padding: 1px; line-height: 1.1;"
                                title="DOF 0: ${angle0}°, DOF 1: ${angle1}°${isAutoExecuteEnabled ? ' (auto-exec)' : ''}">
                            ${angle0}°<br>${angle1}°
                        </button>
                    `);
                });
            });
            
            container.append(multiContainer);
        }
    } else {
        // Original flat layout for joints without sections (KNEE, HIP)
        const buttonContainer = $('<div class="flex flex-wrap gap-2"></div>');
        
        defaultButtons.forEach(btn => {
            // For KNEE: show only DOF 0 angle (DOF 1 doesn't exist)
            // For HIP: keep full notation until prototype is available
            let buttonLabel;
            if (jointType === 'KNEE') {
                buttonLabel = btn.angle0 === 0 ? 'Zero' : btn.angle0 + '°';
            } else {
                // HIP: keep original format for now
                buttonLabel = `${btn.label}<br><span class="text-xs opacity-75">${btn.angle0}°/${btn.angle1}°</span>`;
            }
            
            buttonContainer.append(`
                <button onclick="setMultiDofQuickAngles(${btn.angle0}, ${btn.angle1})" 
                        class="${btn.colorClass} text-white text-xs py-2 rounded"
                        style="min-width: 70px; width: 70px;"
                        title="DOF 0: ${btn.angle0}°, DOF 1: ${btn.angle1}°${isAutoExecuteEnabled ? ' (esecuzione automatica attiva)' : ''}">
                    ${buttonLabel}
                </button>
            `);
        });
        
        container.append(buttonContainer);
    }
    
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
            // DOF 0 only (Plantarflexion/Dorsiflexion) - DOF 1 = 0
            { angle0: -50, angle1: 0, label: '-50°', colorClass: defaultColor, section: 'dof0' },
            { angle0: -40, angle1: 0, label: '-40°', colorClass: defaultColor, section: 'dof0' },
            { angle0: -30, angle1: 0, label: '-30°', colorClass: defaultColor, section: 'dof0' },
            { angle0: -20, angle1: 0, label: '-20°', colorClass: defaultColor, section: 'dof0' },
            { angle0: -10, angle1: 0, label: '-10°', colorClass: defaultColor, section: 'dof0' },
            { angle0: 0, angle1: 0, label: 'Zero', colorClass: baseColor, section: 'dof0' },
            { angle0: 5, angle1: 0, label: '5°', colorClass: defaultColor, section: 'dof0' },
            { angle0: 10, angle1: 0, label: '10°', colorClass: defaultColor, section: 'dof0' },
            { angle0: 15, angle1: 0, label: '15°', colorClass: defaultColor, section: 'dof0' },
            { angle0: 20, angle1: 0, label: '20°', colorClass: defaultColor, section: 'dof0' },
            { angle0: 25, angle1: 0, label: '25°', colorClass: defaultColor, section: 'dof0' },
            
            // DOF 1 only (Inversion/Eversion) - DOF 0 = 0
            { angle0: 0, angle1: -25, label: '-25°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: -20, label: '-20°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: -15, label: '-15°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: -10, label: '-10°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: -5, label: '-5°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: 0, label: 'Zero', colorClass: baseColor, section: 'dof1' },
            { angle0: 0, angle1: 5, label: '5°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: 10, label: '10°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: 15, label: '15°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: 20, label: '20°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            { angle0: 0, angle1: 25, label: '25°', colorClass: 'bg-teal-400 hover:bg-teal-500', section: 'dof1' },
            
            // Multi-DOF combinations - Generated as 2D grid dynamically (see generateDefaultQuickButtons)
            { angle0: 0, angle1: 0, label: 'Placeholder', colorClass: baseColor, section: 'multi' }
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
    
    // Note: autoExecuteToggle event listener is already registered earlier in the code
    // (with mutual exclusion logic for sequence mode)
    
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
 * Uses CAN streaming @ 200Hz when CAN is connected, otherwise uses Serial polling.
 * @param {string} jointType - KNEE, ANKLE, or HIP
 */
function startEncoderTest(jointType) {
    // Stop any active test
    stopEncoderTest();
    
    encoderTestActive = true;
    currentEncoderJointType = jointType;
    
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
    
    // Check if CAN is connected - use CAN streaming via SocketIO (real-time)
    if (canConnectionState && canConnectionState.connected) {
        // Start CAN encoder streaming - data arrives via SocketIO 'encoder_stream' event
        $.ajax({
            url: '/can/encoder_stream/start',
            type: 'POST'
        }).done(response => {
            if (response.status === 'success') {
                appendStatusMessage(`🔄 Encoder streaming started via CAN @ 50Hz for ${jointType} (SocketIO real-time)`);
                encoderStreamingViaCan = true;
                // No polling needed - data arrives via SocketIO 'encoder_stream' event
                // which is handled in the socket.on('encoder_stream', ...) listener
            } else {
                appendStatusMessage(`❌ Failed to start CAN encoder streaming: ${response.message}`);
                fallbackToSerialEncoderTest(jointType);
            }
        }).fail(xhr => {
            appendStatusMessage(`❌ CAN encoder streaming error: ${xhr.responseJSON?.message || 'Unknown error'}`);
            fallbackToSerialEncoderTest(jointType);
        });
    } else {
        // Use Serial polling (fallback)
        fallbackToSerialEncoderTest(jointType);
    }
    
    // Aggiorna UI
    updateEncoderTestUI(true, jointType);
}

/**
 * Fallback to serial encoder test when CAN is not available
 */
function fallbackToSerialEncoderTest(jointType) {
    encoderStreamingViaCan = false;
    
    // Get selected interval for this joint
    const intervalSelector = `#${jointType.toLowerCase()}EncoderInterval`;
    const intervalMs = parseInt($(intervalSelector).val()) || 500;
    
    // Send start command to backend (serial)
    sendCommand('start-test-encoder');
    
    // Start periodic polling
    encoderTestInterval = setInterval(() => {
        if (encoderTestActive) {
            requestEncoderData();
        }
    }, intervalMs);
    
    appendStatusMessage(`🔄 Encoder test started for ${jointType} via Serial (interval: ${intervalMs}ms)`);
}

/**
 * Fetch encoder stream data from CAN
 */
function fetchCanEncoderStreamData() {
    $.ajax({
        url: '/can/encoder_stream/status',
        type: 'GET'
    }).done(response => {
        if (response.status === 'success' && response.data && response.data.length > 0) {
            // Filter data by current joint type
            const expectedPrefix = currentEncoderJointType ? currentEncoderJointType.toUpperCase() : '';
            const filteredData = response.data.filter(point => {
                if (!point.joint_name || !expectedPrefix) return true;  // No filter if missing info
                return point.joint_name.toUpperCase().startsWith(expectedPrefix);
            });
            
            if (filteredData.length === 0) return;  // No data for current joint
            
            // Process each filtered data point
            filteredData.forEach(point => {
                const timestamp = Date.now();
                encoderTestData.timestamps.push(timestamp);
                
                // Process each DOF angle
                point.angles_deg.forEach((angle, dofIndex) => {
                    if (angle !== null && encoderTestData.dofData[dofIndex]) {
                        encoderTestData.dofData[dofIndex].timestamps.push(timestamp);
                        encoderTestData.dofData[dofIndex].values.push(angle);
                    }
                });
            });
            
            // Update chart with last filtered point
            const lastPoint = filteredData[filteredData.length - 1];
            updateEncoderChartFromCanStream(lastPoint);
        }
    });
}

/**
 * Update holding target display for a specific DOF
 * Shows the target angle that the controller is trying to hold
 */
function updateHoldingTargetDisplay(dof, angle) {
    const jointType = $("#jointSelect").val()?.split('_')[0]?.toLowerCase() || '';
    
    // Update the holding target indicator
    const indicatorId = `holdingTarget${jointType}Dof${dof}`;
    let indicator = document.getElementById(indicatorId);
    
    // If indicator doesn't exist, create it dynamically near the encoder display
    if (!indicator) {
        // Try to find the encoder display span for this DOF
        const encoderSpanId = `${jointType}EncoderDof${dof}`;
        const encoderSpan = document.getElementById(encoderSpanId);
        
        if (encoderSpan && encoderSpan.parentElement) {
            // Create holding target indicator
            indicator = document.createElement('span');
            indicator.id = indicatorId;
            indicator.className = 'ml-2 text-xs font-medium text-purple-600 bg-purple-100 px-2 py-0.5 rounded';
            indicator.title = 'Target angle being held by controller';
            encoderSpan.parentElement.appendChild(indicator);
        }
    }
    
    if (indicator) {
        indicator.innerHTML = `🎯 ${angle.toFixed(2)}°`;
        
        // Flash animation to show update
        indicator.classList.add('animate-pulse');
        setTimeout(() => indicator.classList.remove('animate-pulse'), 500);
    }
    
    // Also log to console for debugging
    console.log(`[HOLDING] DOF ${dof} → ${angle.toFixed(2)}°`);
}

// ============================================================================
// PID DIAGNOSTICS FUNCTIONS
// ============================================================================

let pidDiagStartTime = null;

/**
 * Start PID diagnostics streaming
 */
function startPidDiagStream() {
    $.ajax({
        url: '/can/pid_diag/start',
        type: 'POST'
    }).done(response => {
        if (response.status === 'success') {
            pidDiagStreamActive = true;
            pidDiagStartTime = Date.now();
            appendStatusMessage('📊 PID diagnostics streaming started @ 20Hz');
            $('#startPidDiagBtn').prop('disabled', true);
            $('#stopPidDiagBtn').prop('disabled', false);
        } else {
            appendStatusMessage(`❌ Failed to start PID diagnostics: ${response.message}`);
        }
    }).fail(xhr => {
        appendStatusMessage(`❌ PID diagnostics error: ${xhr.responseJSON?.message || 'Unknown error'}`);
    });
}

/**
 * Stop PID diagnostics streaming
 */
function stopPidDiagStream() {
    $.ajax({
        url: '/can/pid_diag/stop',
        type: 'POST'
    }).done(response => {
        pidDiagStreamActive = false;
        appendStatusMessage('📊 PID diagnostics streaming stopped');
        $('#startPidDiagBtn').prop('disabled', false);
        $('#stopPidDiagBtn').prop('disabled', true);
    }).fail(xhr => {
        appendStatusMessage(`❌ PID diagnostics stop error: ${xhr.responseJSON?.message || 'Unknown error'}`);
    });
}

/**
 * Update PID diagnostics streaming frequency
 * Higher frequency = more data points for charts and CSV
 */
function updatePidDiagFrequency() {
    const freqHz = parseInt($('#pidDiagFrequency').val()) || 20;
    const intervalMs = 1000 / freqHz;
    
    $.ajax({
        url: '/can/pid_diag_frequency',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ freq_hz: freqHz })
    }).done(response => {
        if (response.status === 'success') {
            $('#pidDiagFreqInfo').text(`= ${intervalMs.toFixed(0)}ms interval`);
            appendStatusMessage(`📊 PID diagnostics frequency set to ${freqHz}Hz`);
        } else {
            appendStatusMessage(`❌ Failed to set frequency: ${response.message}`, 'error');
        }
    }).fail(xhr => {
        appendStatusMessage(`❌ Frequency update error: ${xhr.responseJSON?.message || 'Unknown error'}`, 'error');
    });
}

/**
 * Clear PID diagnostics charts
 */
function clearPidDiagCharts() {
    if (pidErrorChart) {
        pidErrorChart.data.datasets.forEach(ds => ds.data = []);
        pidErrorChart.update('none');
    }
    if (pidTorqueChart) {
        pidTorqueChart.data.datasets.forEach(ds => ds.data = []);
        pidTorqueChart.update('none');
    }
    pidDiagStartTime = Date.now();
    pidDiagDataBuffer = [];  // Clear export buffer too
    $('#pidDiagBufferCount').text('0');
    appendStatusMessage('📊 PID diagnostics charts and buffer cleared');
}

/**
 * Export PID diagnostics data to CSV file
 */
function exportPidDiagToCSV() {
    if (pidDiagDataBuffer.length === 0) {
        appendStatusMessage('⚠️ No PID data to export. Start diagnostics and perform a movement first.');
        return;
    }
    
    // CSV header
    const headers = [
        'time_s',
        'target_dof0', 'current_dof0', 'error_dof0', 'torque_a_dof0', 'torque_b_dof0',
        'target_dof1', 'current_dof1', 'error_dof1', 'torque_a_dof1', 'torque_b_dof1'
    ];
    
    // Build CSV content
    let csvContent = headers.join(',') + '\n';
    
    pidDiagDataBuffer.forEach(row => {
        const values = [
            row.time_s,
            row.target_dof0.toFixed(2), row.current_dof0.toFixed(2), row.error_dof0.toFixed(2),
            row.torque_a_dof0.toFixed(1), row.torque_b_dof0.toFixed(1),
            row.target_dof1.toFixed(2), row.current_dof1.toFixed(2), row.error_dof1.toFixed(2),
            row.torque_a_dof1.toFixed(1), row.torque_b_dof1.toFixed(1)
        ];
        csvContent += values.join(',') + '\n';
    });
    
    // Create download link
    const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' });
    const link = document.createElement('a');
    const url = URL.createObjectURL(blob);
    
    // Generate filename with timestamp
    const now = new Date();
    const timestamp = now.toISOString().replace(/[:.]/g, '-').slice(0, 19);
    const joint = $('#jointSelect').val() || 'JOINT';
    const filename = `pid_diag_${joint}_${timestamp}.csv`;
    
    link.setAttribute('href', url);
    link.setAttribute('download', filename);
    link.style.visibility = 'hidden';
    document.body.appendChild(link);
    link.click();
    document.body.removeChild(link);
    
    appendStatusMessage(`📥 Exported ${pidDiagDataBuffer.length} records to ${filename}`);
}

// ============================================================================
// MOVEMENT METRICS DISPLAY
// ============================================================================

// History of scores for each DOF
let metricsHistory = [];
const MAX_METRICS_HISTORY = 10;

/**
 * Update the movement metrics display when new data arrives
 */
function updateMovementMetricsDisplay(data) {
    if (!data) return;
    
    const dof = data.dof;
    const score = data.score || 0;
    
    // Get color based on score
    const getScoreColor = (s) => {
        if (s >= 80) return 'text-green-500';
        if (s >= 60) return 'text-yellow-500';
        if (s >= 40) return 'text-orange-500';
        return 'text-red-500';
    };
    
    const getBgColor = (s) => {
        if (s >= 80) return 'border-green-300 bg-green-50';
        if (s >= 60) return 'border-yellow-300 bg-yellow-50';
        if (s >= 40) return 'border-orange-300 bg-orange-50';
        return 'border-red-300 bg-red-50';
    };
    
    // Update the appropriate DOF card
    const prefix = `#metricsDof${dof}`;
    
    // Update score with color
    $(prefix + 'Score')
        .text(score + '/100')
        .removeClass('text-gray-400 text-green-500 text-yellow-500 text-orange-500 text-red-500')
        .addClass(getScoreColor(score));
    
    // Update card border color
    $(prefix)
        .removeClass('border-gray-200 border-green-300 border-yellow-300 border-orange-300 border-red-300 bg-gray-50 bg-green-50 bg-yellow-50 bg-orange-50 bg-red-50')
        .addClass(getBgColor(score));
    
    // Update individual metrics
    $(prefix + 'Rise').text(data.rise_time_ms + 'ms');
    $(prefix + 'Settle').text(data.settling_time_ms + 'ms');
    $(prefix + 'Overshoot').text(data.overshoot_pct?.toFixed(1) + '%');
    $(prefix + 'Sse').text(data.sse_deg?.toFixed(2) + '°');
    $(prefix + 'Torque').text(Math.max(Math.abs(data.max_torque_A || 0), Math.abs(data.max_torque_B || 0)));
    $(prefix + 'Duration').text(data.duration_ms + 'ms');
    
    // Add to history (include full metrics data for auto-tune analysis)
    metricsHistory.unshift({
        dof: dof,
        score: score,
        rise_time_ms: data.rise_time_ms,
        settling_time_ms: data.settling_time_ms,
        overshoot_pct: data.overshoot_pct,
        sse_deg: data.sse_deg,
        max_torque: Math.max(Math.abs(data.max_torque_A || 0), Math.abs(data.max_torque_B || 0)),
        duration_ms: data.duration_ms,
        timestamp: new Date().toLocaleTimeString()
    });
    
    // Trim history
    if (metricsHistory.length > MAX_METRICS_HISTORY) {
        metricsHistory = metricsHistory.slice(0, MAX_METRICS_HISTORY);
    }
    
    // Update history display
    updateMetricsHistoryDisplay();
    
    // Log to status
    appendStatusMessage(`🏆 DOF ${dof} movement complete: Score ${score}/100 (rise=${data.rise_time_ms}ms, settle=${data.settling_time_ms}ms)`);
}

/**
 * Update the metrics history display
 */
function updateMetricsHistoryDisplay() {
    const $history = $('#metricsHistory');
    
    if (metricsHistory.length === 0) {
        $history.html('<span class="text-gray-400">No data yet</span>');
        return;
    }
    
    const getScoreBadgeClass = (score) => {
        if (score >= 80) return 'bg-green-500';
        if (score >= 60) return 'bg-yellow-500';
        if (score >= 40) return 'bg-orange-500';
        return 'bg-red-500';
    };
    
    const badges = metricsHistory.map((m, i) => {
        const isLatest = i === 0;
        const sizeClass = isLatest ? 'px-3 py-1 text-sm' : 'px-2 py-0.5';
        const ringClass = isLatest ? 'ring-2 ring-offset-1 ring-gray-400' : '';
        return `<span class="inline-flex items-center ${getScoreBadgeClass(m.score)} text-white rounded ${sizeClass} ${ringClass}" title="DOF${m.dof} @ ${m.timestamp}">
            D${m.dof}: ${m.score}
        </span>`;
    }).join('');
    
    $history.html(badges);
}

// ============================================================================
// OSCILLATION TEST FOR PID TUNING
// ============================================================================

let oscTestActive = false;
let oscTestTimer = null;
let oscTestCurrentRep = 0;
let oscTestTotalReps = 0;
let oscTestCurrentPoint = 'A';  // 'A' or 'B'
let oscTestMetricsReceived = 0;

/**
 * Start oscillation test between two points
 */
function startOscillationTest() {
    const dof = parseInt($('#oscTestDof').val());
    const pointA = parseFloat($('#oscTestPointA').val());
    const pointB = parseFloat($('#oscTestPointB').val());
    const pauseMs = parseInt($('#oscTestPause').val());
    const moveTimeMs = parseInt($('#oscTestMoveTime').val());
    const reps = parseInt($('#oscTestReps').val());
    const interpolationMode = $('#oscTestInterpolation').val() || 'linear';
    
    // Validation
    if (isNaN(pointA) || isNaN(pointB) || pointA === pointB) {
        appendStatusMessage('❌ Invalid points: A and B must be different');
        return;
    }
    
    // Set interpolation mode via CAN before starting
    $.ajax({
        url: '/can/interpolation_mode',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify({ mode: interpolationMode })
    }).done(response => {
        const modeLabel = interpolationMode === 'cosine' ? 'SMOOTH' : 'LINEAR';
        appendStatusMessage(`⚙️ Interpolation mode set to: ${modeLabel}`);
        
        // Now start the oscillation test
        oscTestActive = true;
        oscTestCurrentRep = 0;
        oscTestTotalReps = reps;
        oscTestCurrentPoint = 'A';
        oscTestMetricsReceived = 0;
        
        // Update UI
        $('#oscTestStartBtn').prop('disabled', true);
        $('#oscTestStopBtn').prop('disabled', false);
        updateOscTestStatus(`Starting... 0/${reps} reps`);
        
        appendStatusMessage(`🔄 Oscillation test started: DOF${dof} ${pointA}° ↔ ${pointB}° × ${reps} reps (${modeLabel})`);
        
        // Start the oscillation loop
        oscTestLoop(dof, pointA, pointB, pauseMs, moveTimeMs);
    }).fail(xhr => {
        appendStatusMessage(`❌ Failed to set interpolation mode: ${xhr.responseJSON?.message || 'Unknown error'}`);
    });
}

/**
 * Main oscillation loop
 */
function oscTestLoop(dof, pointA, pointB, pauseMs, moveTimeMs) {
    if (!oscTestActive) return;
    
    if (oscTestCurrentRep >= oscTestTotalReps) {
        // Test complete
        stopOscillationTest(true);
        return;
    }
    
    // Determine target angle
    const targetAngle = oscTestCurrentPoint === 'A' ? pointA : pointB;
    const direction = oscTestCurrentPoint === 'A' ? '→A' : '→B';
    
    updateOscTestStatus(`Rep ${oscTestCurrentRep + 1}/${oscTestTotalReps} ${direction} (${targetAngle}°)`);
    
    // Send waypoint
    sendOscTestWaypoint(dof, targetAngle, moveTimeMs);
    
    // Schedule next movement after pause
    const totalWait = moveTimeMs + pauseMs;
    oscTestTimer = setTimeout(() => {
        // Toggle point
        if (oscTestCurrentPoint === 'A') {
            oscTestCurrentPoint = 'B';
        } else {
            oscTestCurrentPoint = 'A';
            oscTestCurrentRep++;  // Count one A→B→A as one rep
        }
        
        // Continue loop
        oscTestLoop(dof, pointA, pointB, pauseMs, moveTimeMs);
    }, totalWait);
}

/**
 * Send a single waypoint for oscillation test
 */
function sendOscTestWaypoint(dof, angle, moveTimeMs) {
    const joint = $('#jointSelect').val() || 'ANKLE_RIGHT';
    
    // Build angles array with null for unused DOFs
    const angles = [null, null, null];
    angles[dof] = angle;
    
    const data = {
        joint: joint,
        angles_deg: angles,
        t_offset_ms: moveTimeMs
    };
    
    $.ajax({
        url: '/can/waypoint',
        type: 'POST',
        contentType: 'application/json',
        data: JSON.stringify(data)
    }).done(response => {
        console.log(`Osc waypoint sent: DOF${dof} → ${angle}°`);
    }).fail(xhr => {
        console.error('Osc waypoint failed:', xhr.responseJSON);
        appendStatusMessage(`❌ Oscillation waypoint failed`);
    });
}

/**
 * Stop oscillation test
 */
function stopOscillationTest(completed = false) {
    oscTestActive = false;
    
    if (oscTestTimer) {
        clearTimeout(oscTestTimer);
        oscTestTimer = null;
    }
    
    // Update UI
    $('#oscTestStartBtn').prop('disabled', false);
    $('#oscTestStopBtn').prop('disabled', true);
    
    if (completed) {
        updateOscTestStatus(`✅ Complete! ${oscTestTotalReps} reps, ${metricsHistory.length} metrics`);
        appendStatusMessage(`✅ Oscillation test complete: ${oscTestTotalReps} reps`);
    } else {
        updateOscTestStatus(`⏹️ Stopped at rep ${oscTestCurrentRep + 1}/${oscTestTotalReps}`);
        appendStatusMessage(`⏹️ Oscillation test stopped`);
    }
}

/**
 * Reset oscillation test state
 */
function resetOscillationTest() {
    stopOscillationTest();
    updateOscTestStatus('<span class="text-gray-400">Ready</span>');
    
    // Optionally go to center position
    const pointA = parseFloat($('#oscTestPointA').val());
    const pointB = parseFloat($('#oscTestPointB').val());
    const center = (pointA + pointB) / 2;
    const dof = parseInt($('#oscTestDof').val());
    
    sendOscTestWaypoint(dof, center, 500);
    appendStatusMessage(`🔄 Reset to center: ${center}°`);
}

/**
 * Handle interpolation mode change - show warning for smooth mode
 */
function onInterpolationModeChange() {
    const mode = $('#oscTestInterpolation').val();
    if (mode === 'cosine') {
        $('#oscTestSmoothWarning').show();
    } else {
        $('#oscTestSmoothWarning').hide();
    }
}

/**
 * Update oscillation test status display
 */
function updateOscTestStatus(html) {
    $('#oscTestStatus').html(html);
}

// ============================================================================
// AUTO-TUNE SUGGESTIONS
// ============================================================================

// Store current suggestions for later application
let currentAutoTuneSuggestions = null;
let autoTuneLoopActive = false;
let autoTuneLoopTimer = null;
let autoTuneIterations = 0;
const MAX_AUTO_TUNE_ITERATIONS = 10;

// Thresholds for problem detection
const AUTO_TUNE_THRESHOLDS = {
    overshoot_high: 5.0,        // % - above this is problematic
    overshoot_very_high: 15.0,  // % - major overshoot
    rise_time_slow: 500,        // ms
    settling_slow: 800,         // ms
    sse_high: 0.3,              // degrees
    sse_very_high: 0.5,         // degrees
    score_good: 75              // Above this, PID is considered well-tuned
};

// Adjustment factors for PID parameters
const AUTO_TUNE_ADJUSTMENTS = {
    overshoot_high: { kp: -0.10, kd: +0.20 },      // Reduce Kp, increase Kd
    overshoot_very_high: { kp: -0.20, kd: +0.30 }, // More aggressive
    rise_time_slow: { kp: +0.15 },                  // Increase Kp
    settling_slow: { kd: +0.15 },                   // Increase Kd
    sse_high: { ki: +0.25 },                        // Increase Ki
    sse_very_high: { ki: +0.50 },                   // More aggressive Ki
    oscillating: { kp: -0.15, kd: +0.25, ki: -0.10 } // Dampen oscillations
};

/**
 * Analyze recent metrics and generate PID tuning suggestions
 */
function analyzeMetricsAndSuggest() {
    const sampleSize = parseInt($('#autoTuneSampleSize').val()) || 3;
    
    // Get recent metrics for DOF 0 (primary tuning target)
    // Filter by most recent DOF or allow DOF selection
    const targetDof = parseInt($('#oscTestDof').val()) || 0;
    
    const recentMetrics = metricsHistory
        .filter(m => m.dof === targetDof)
        .slice(0, sampleSize);
    
    if (recentMetrics.length === 0) {
        appendStatusMessage('⚠️ No metrics available for DOF ' + targetDof + '. Run some movements first.');
        return;
    }
    
    // Calculate averages from full metrics data
    const avgScore = recentMetrics.reduce((sum, m) => sum + (m.score || 0), 0) / recentMetrics.length;
    const riseTime = recentMetrics.reduce((sum, m) => sum + (m.rise_time_ms || 0), 0) / recentMetrics.length;
    const overshoot = recentMetrics.reduce((sum, m) => sum + (m.overshoot_pct || 0), 0) / recentMetrics.length;
    const settling = recentMetrics.reduce((sum, m) => sum + (m.settling_time_ms || 0), 0) / recentMetrics.length;
    const sse = recentMetrics.reduce((sum, m) => sum + (m.sse_deg || 0), 0) / recentMetrics.length;
    
    appendStatusMessage(`📊 Analyzing ${recentMetrics.length} samples for DOF ${targetDof}`);
    
    // Update analysis display
    $('#autoTuneAvgScore').text(avgScore.toFixed(1)).removeClass('text-gray-400')
        .addClass(avgScore >= 75 ? 'text-green-600' : (avgScore >= 50 ? 'text-yellow-600' : 'text-red-600'));
    $('#autoTuneAvgRise').text(riseTime + 'ms').removeClass('text-gray-400');
    $('#autoTuneAvgOvershoot').text(overshoot.toFixed(1) + '%').removeClass('text-gray-400');
    $('#autoTuneAvgSettling').text(settling + 'ms').removeClass('text-gray-400');
    $('#autoTuneAvgSse').text(sse.toFixed(2) + '°').removeClass('text-gray-400');
    
    // Detect issues and generate suggestions
    const issues = [];
    const adjustments = { kp: 0, ki: 0, kd: 0 };
    
    // Check overshoot
    if (overshoot > AUTO_TUNE_THRESHOLDS.overshoot_very_high) {
        issues.push({ icon: '🚨', text: `Very high overshoot (${overshoot.toFixed(1)}%)`, severity: 'high' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.overshoot_very_high).forEach(([k, v]) => adjustments[k] += v);
    } else if (overshoot > AUTO_TUNE_THRESHOLDS.overshoot_high) {
        issues.push({ icon: '⚠️', text: `High overshoot (${overshoot.toFixed(1)}%)`, severity: 'medium' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.overshoot_high).forEach(([k, v]) => adjustments[k] += v);
    }
    
    // Check rise time
    if (riseTime > AUTO_TUNE_THRESHOLDS.rise_time_slow) {
        issues.push({ icon: '🐢', text: `Slow rise time (${riseTime}ms)`, severity: 'medium' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.rise_time_slow).forEach(([k, v]) => adjustments[k] += v);
    }
    
    // Check settling time
    if (settling > AUTO_TUNE_THRESHOLDS.settling_slow) {
        issues.push({ icon: '⏳', text: `Slow settling (${settling}ms)`, severity: 'medium' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.settling_slow).forEach(([k, v]) => adjustments[k] += v);
    }
    
    // Check steady-state error
    if (sse > AUTO_TUNE_THRESHOLDS.sse_very_high) {
        issues.push({ icon: '🎯', text: `High steady-state error (${sse.toFixed(2)}°)`, severity: 'high' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.sse_very_high).forEach(([k, v]) => adjustments[k] += v);
    } else if (sse > AUTO_TUNE_THRESHOLDS.sse_high) {
        issues.push({ icon: '🎯', text: `Moderate SSE (${sse.toFixed(2)}°)`, severity: 'medium' });
        Object.entries(AUTO_TUNE_ADJUSTMENTS.sse_high).forEach(([k, v]) => adjustments[k] += v);
    }
    
    // If score is good and no major issues, show "well tuned" message
    if (issues.length === 0 || avgScore >= AUTO_TUNE_THRESHOLDS.score_good) {
        $('#autoTuneSuggestionsPanel').hide();
        $('#autoTuneNoSuggestions').show();
        currentAutoTuneSuggestions = null;
        appendStatusMessage('✅ PID analysis: No significant issues detected');
        return;
    }
    
    // Get current PID parameters
    const currentKp = parseFloat($(`#outerPidDof${targetDof}Kp`).val()) || 15;
    const currentKi = parseFloat($(`#outerPidDof${targetDof}Ki`).val()) || 0.01;
    const currentKd = parseFloat($(`#outerPidDof${targetDof}Kd`).val()) || 0.05;
    
    // Calculate suggested values with bounds
    const suggestedKp = Math.max(0.1, Math.min(50, currentKp * (1 + adjustments.kp)));
    const suggestedKi = Math.max(0.001, Math.min(1, currentKi * (1 + adjustments.ki)));
    const suggestedKd = Math.max(0, Math.min(2, currentKd * (1 + adjustments.kd)));
    
    // Build suggestions object
    currentAutoTuneSuggestions = {
        dof: targetDof,
        current: { kp: currentKp, ki: currentKi, kd: currentKd },
        suggested: { kp: suggestedKp, ki: suggestedKi, kd: suggestedKd },
        adjustments: adjustments,
        issues: issues
    };
    
    // Update UI
    displayAutoTuneSuggestions(currentAutoTuneSuggestions);
    
    appendStatusMessage(`🔍 Analysis complete: ${issues.length} issue(s) detected`);
}

/**
 * Display auto-tune suggestions in the UI
 */
function displayAutoTuneSuggestions(suggestions) {
    $('#autoTuneNoSuggestions').hide();
    $('#autoTuneSuggestionsPanel').show();
    $('#autoTuneDofLabel').text(`DOF ${suggestions.dof}`);
    
    // Build issues list
    const issuesHtml = suggestions.issues.map(issue => {
        const bgColor = issue.severity === 'high' ? 'bg-red-100 text-red-800' : 'bg-yellow-100 text-yellow-800';
        return `<div class="p-1 ${bgColor} rounded">${issue.icon} ${issue.text}</div>`;
    }).join('');
    $('#autoTuneIssues').html(issuesHtml);
    
    // Build parameters table
    const formatChange = (current, suggested) => {
        const pctChange = ((suggested - current) / current * 100).toFixed(0);
        const arrow = pctChange > 0 ? '↑' : '↓';
        const color = pctChange > 0 ? 'text-green-600' : 'text-red-600';
        return `<span class="${color}">${arrow}${Math.abs(pctChange)}%</span>`;
    };
    
    const tableRows = [
        { name: 'Kp', current: suggestions.current.kp, suggested: suggestions.suggested.kp, digits: 3 },
        { name: 'Ki', current: suggestions.current.ki, suggested: suggestions.suggested.ki, digits: 4 },
        { name: 'Kd', current: suggestions.current.kd, suggested: suggestions.suggested.kd, digits: 4 }
    ].filter(row => row.current !== row.suggested).map(row => {
        const changeHtml = formatChange(row.current, row.suggested);
        return `<tr class="border-b border-amber-100">
            <td class="px-2 py-1 font-semibold">${row.name}</td>
            <td class="px-2 py-1 text-right font-mono">${row.current.toFixed(row.digits)}</td>
            <td class="px-2 py-1 text-center">${changeHtml}</td>
            <td class="px-2 py-1 text-right font-mono font-bold text-amber-700">${row.suggested.toFixed(row.digits)}</td>
        </tr>`;
    }).join('');
    
    $('#autoTuneSuggestionsTable').html(tableRows);
}

/**
 * Apply the current auto-tune suggestions to PID parameters
 */
function applyAutoTuneSuggestions() {
    if (!currentAutoTuneSuggestions) {
        appendStatusMessage('⚠️ No suggestions to apply');
        return;
    }
    
    const dof = currentAutoTuneSuggestions.dof;
    const suggested = currentAutoTuneSuggestions.suggested;
    
    // Update UI input fields
    $(`#outerPidDof${dof}Kp`).val(suggested.kp.toFixed(4));
    $(`#outerPidDof${dof}Ki`).val(suggested.ki.toFixed(4));
    $(`#outerPidDof${dof}Kd`).val(suggested.kd.toFixed(4));
    
    // Save to device
    savePidParams(dof);
    
    appendStatusMessage(`✅ Applied PID suggestions for DOF ${dof}: Kp=${suggested.kp.toFixed(3)}, Ki=${suggested.ki.toFixed(4)}, Kd=${suggested.kd.toFixed(4)}`);
    
    // Hide suggestions panel after applying
    dismissAutoTuneSuggestions();
}

/**
 * Dismiss suggestions without applying
 */
function dismissAutoTuneSuggestions() {
    currentAutoTuneSuggestions = null;
    $('#autoTuneSuggestionsPanel').hide();
}

/**
 * Run one iteration of auto-tune: Apply suggestions and run test
 */
function runAutoTuneIteration() {
    if (!currentAutoTuneSuggestions) {
        appendStatusMessage('⚠️ No suggestions to test. Run analysis first.');
        return;
    }
    
    // Apply suggestions
    applyAutoTuneSuggestions();
    
    // Wait a moment for PID to update, then run oscillation test
    setTimeout(() => {
        // Clear previous metrics for clean comparison
        metricsHistory = [];
        updateMovementMetricsHistory();
        
        // Start oscillation test
        appendStatusMessage('🔄 Starting verification test...');
        startOscillationTest();
        
        // After test completes, auto-analyze
        // We'll use the existing test completion mechanism
        // Set up a check to analyze when test is done
        const checkTestComplete = setInterval(() => {
            if (!oscTestActive) {
                clearInterval(checkTestComplete);
                // Wait for metrics to arrive
                setTimeout(() => {
                    appendStatusMessage('📊 Re-analyzing after parameter change...');
                    analyzeMetricsAndSuggest();
                }, 1000);
            }
        }, 500);
        
    }, 500);
}

/**
 * Start automatic tuning loop
 */
function startAutoTuneLoop() {
    if (autoTuneLoopActive) return;
    
    autoTuneLoopActive = true;
    autoTuneIterations = 0;
    
    $('#autoTuneLoopStatus').show();
    updateAutoTuneLoopProgress('Starting auto-tune loop...');
    
    autoTuneLoopStep();
}

/**
 * One step of the auto-tune loop
 */
function autoTuneLoopStep() {
    if (!autoTuneLoopActive) return;
    
    if (autoTuneIterations >= MAX_AUTO_TUNE_ITERATIONS) {
        stopAutoTuneLoop('Max iterations reached');
        return;
    }
    
    autoTuneIterations++;
    updateAutoTuneLoopProgress(`Iteration ${autoTuneIterations}/${MAX_AUTO_TUNE_ITERATIONS}`);
    
    // Run analysis
    analyzeMetricsAndSuggest();
    
    // Check if we have suggestions
    if (!currentAutoTuneSuggestions) {
        stopAutoTuneLoop('PID well-tuned, no more adjustments needed');
        return;
    }
    
    // Apply and test
    runAutoTuneIteration();
    
    // Schedule next iteration after test completes
    const checkComplete = setInterval(() => {
        if (!oscTestActive) {
            clearInterval(checkComplete);
            // Continue loop after short delay
            autoTuneLoopTimer = setTimeout(() => {
                autoTuneLoopStep();
            }, 2000);
        }
    }, 500);
}

/**
 * Stop auto-tune loop
 */
function stopAutoTuneLoop(reason = 'Stopped by user') {
    autoTuneLoopActive = false;
    
    if (autoTuneLoopTimer) {
        clearTimeout(autoTuneLoopTimer);
        autoTuneLoopTimer = null;
    }
    
    $('#autoTuneLoopStatus').hide();
    appendStatusMessage(`⏹️ Auto-tune loop stopped: ${reason} (${autoTuneIterations} iterations)`);
}

/**
 * Update auto-tune loop progress display
 */
function updateAutoTuneLoopProgress(text) {
    $('#autoTuneLoopProgress').text(text);
}

/**
 * Update PID diag display (target/error values)
 */
function updatePidDiagDisplay(data) {
    if (!data) return;
    
    // Update target displays
    if (data.target_deg) {
        $('#pidDiagTarget0').text(data.target_deg[0]?.toFixed(2) + '°' || '--°');
        $('#pidDiagTarget1').text(data.target_deg[1]?.toFixed(2) + '°' || '--°');
    }
    
    // Update error displays
    if (data.error_deg) {
        const error0 = data.error_deg[0];
        const error1 = data.error_deg[1];
        
        // Color code based on error magnitude
        const formatError = (err) => {
            if (err === undefined || err === null) return '--°';
            const absErr = Math.abs(err);
            const color = absErr < 0.5 ? 'text-green-600' : (absErr < 2 ? 'text-orange-600' : 'text-red-600');
            return `<span class="${color}">${err.toFixed(2)}°</span>`;
        };
        
        $('#pidDiagError0').html(formatError(error0));
        $('#pidDiagError1').html(formatError(error1));
    }
}

/**
 * Update PID torque display
 */
function updatePidTorqueDisplay(data) {
    if (!data) return;
    
    if (data.torque_A) {
        $('#pidDiagTorqueA0').text(data.torque_A[0] || '--');
        $('#pidDiagTorqueA1').text(data.torque_A[1] || '--');
    }
    if (data.torque_B) {
        $('#pidDiagTorqueB0').text(data.torque_B[0] || '--');
        $('#pidDiagTorqueB1').text(data.torque_B[1] || '--');
    }
}

/**
 * Update PID error chart with new data
 */
function updatePidErrorChart(data) {
    if (!pidErrorChart || !data.error_deg) return;
    
    const timeSeconds = (Date.now() - pidDiagStartTime) / 1000;
    
    // Add data points
    pidErrorChart.data.datasets[0].data.push({ x: timeSeconds, y: data.error_deg[0] });
    pidErrorChart.data.datasets[1].data.push({ x: timeSeconds, y: data.error_deg[1] });
    
    // Limit data points
    const maxPoints = 300;
    pidErrorChart.data.datasets.forEach(ds => {
        while (ds.data.length > maxPoints) ds.data.shift();
    });
    
    // Update x-axis range to follow data
    if (timeSeconds > 30) {
        pidErrorChart.options.scales.x.min = timeSeconds - 30;
        pidErrorChart.options.scales.x.max = timeSeconds;
    }
    
    // Throttle updates
    if (!window.lastPidErrorUpdate || Date.now() - window.lastPidErrorUpdate > 100) {
        pidErrorChart.update('none');
        window.lastPidErrorUpdate = Date.now();
    }
}

/**
 * Update PID torque chart with new data
 */
function updatePidTorqueChart(data) {
    if (!pidTorqueChart || !data.torque_A || !data.torque_B) return;
    
    const timeSeconds = (Date.now() - pidDiagStartTime) / 1000;
    
    // Add data points (4 datasets: DOF0 A, DOF0 B, DOF1 A, DOF1 B)
    pidTorqueChart.data.datasets[0].data.push({ x: timeSeconds, y: data.torque_A[0] });
    pidTorqueChart.data.datasets[1].data.push({ x: timeSeconds, y: data.torque_B[0] });
    pidTorqueChart.data.datasets[2].data.push({ x: timeSeconds, y: data.torque_A[1] });
    pidTorqueChart.data.datasets[3].data.push({ x: timeSeconds, y: data.torque_B[1] });
    
    // Limit data points
    const maxPoints = 300;
    pidTorqueChart.data.datasets.forEach(ds => {
        while (ds.data.length > maxPoints) ds.data.shift();
    });
    
    // Update x-axis range to follow data
    if (timeSeconds > 30) {
        pidTorqueChart.options.scales.x.min = timeSeconds - 30;
        pidTorqueChart.options.scales.x.max = timeSeconds;
    }
    
    // Throttle updates
    if (!window.lastPidTorqueUpdate || Date.now() - window.lastPidTorqueUpdate > 100) {
        pidTorqueChart.update('none');
        window.lastPidTorqueUpdate = Date.now();
    }
}

/**
 * Update encoder display from CAN stream data
 * Updates both the numeric display and the chart
 */
function updateEncoderChartFromCanStream(dataPoint) {
    if (!dataPoint || !dataPoint.angles_deg || !currentEncoderJointType) return;
    
    const jointType = currentEncoderJointType.toLowerCase();
    
    // Update numeric displays (e.g., kneeEncoderDof0, ankleEncoderDof1, etc.)
    dataPoint.angles_deg.forEach((angle, dofIndex) => {
        const spanId = `${jointType}EncoderDof${dofIndex}`;
        const span = document.getElementById(spanId);
        if (span && angle !== null) {
            span.textContent = angle.toFixed(2) + ' °';
        }
    });
    
    // Update chart - chart name is "kneeChart", "ankleChart", etc.
    // For KNEE there's one chart, for ANKLE/HIP there might be per-DOF charts
    if (jointType === 'knee') {
        updateKneeChartFromStream(dataPoint);
    } else if (jointType === 'ankle') {
        updateAnkleChartFromStream(dataPoint);
    } else if (jointType === 'hip') {
        updateHipChartFromStream(dataPoint);
    }
}

/**
 * Update knee chart from stream data
 * Note: kneeChart uses x:linear format with {x, y} data points
 */
function updateKneeChartFromStream(dataPoint) {
    if (typeof kneeChart === 'undefined') return;
    
    // Use time in seconds as X value
    if (!window.kneeChartStartTime) {
        window.kneeChartStartTime = Date.now();
    }
    const timeSeconds = (Date.now() - window.kneeChartStartTime) / 1000;
    
    // DOF 0 is the only DOF for knee - add to first dataset (Encoder angle)
    if (dataPoint.angles_deg[0] !== null) {
        // Use dataset 0 for encoder data
        kneeChart.data.datasets[0].data.push({
            x: timeSeconds,
            y: dataPoint.angles_deg[0]
        });
        
        // Keep only last 100 points
        if (kneeChart.data.datasets[0].data.length > 100) {
            kneeChart.data.datasets[0].data.shift();
        }
    }
    
    // Throttle chart updates to ~10Hz for performance
    if (!window.lastKneeChartUpdate || Date.now() - window.lastKneeChartUpdate > 100) {
        kneeChart.update('none');  // No animation for performance
        window.lastKneeChartUpdate = Date.now();
    }
}

/**
 * Update ankle charts from stream data
 * Throttled to ~10Hz to avoid overwhelming Chart.js
 * Uses {x, y} format for linear x-axis (time in seconds)
 */
function updateAnkleChartFromStream(dataPoint) {
    // Ankle has 2 DOFs with separate charts: ankleDof0Chart, ankleDof1Chart
    // Note: These are global let variables, not window properties
    const charts = [ankleDof0Chart, ankleDof1Chart];
    
    // Initialize start time if not set
    if (!window.ankleChartStartTime) {
        window.ankleChartStartTime = Date.now();
    }
    
    // Calculate time in seconds since start
    const now = Date.now();
    const timeSeconds = (now - window.ankleChartStartTime) / 1000;
    
    // Throttle chart updates to ~10Hz (100ms interval)
    if (!window.lastAnkleChartUpdate) window.lastAnkleChartUpdate = 0;
    const shouldUpdateChart = (now - window.lastAnkleChartUpdate) > 100;
    
    charts.forEach((chartVar, dofIndex) => {
        if (chartVar && dataPoint.angles_deg[dofIndex] !== null) {
            // Add data point as {x, y} for linear x-axis
            chartVar.data.datasets[0].data.push({
                x: timeSeconds,
                y: dataPoint.angles_deg[dofIndex]
            });
            
            // Limit buffer size to 500 points
            if (chartVar.data.datasets[0].data.length > 500) {
                chartVar.data.datasets[0].data.shift();
            }
        }
    });
    
    // Only update chart rendering at throttled rate
    if (shouldUpdateChart) {
        charts.forEach(chartVar => {
            if (chartVar) chartVar.update('none');
        });
        window.lastAnkleChartUpdate = now;
    }
}

/**
 * Update hip charts from stream data
 * Throttled to ~10Hz to avoid overwhelming Chart.js
 * Uses {x, y} format for linear x-axis (time in seconds)
 */
function updateHipChartFromStream(dataPoint) {
    // Hip has 2 DOFs with separate charts: hipDof0Chart, hipDof1Chart
    // Note: These are global let variables, not window properties
    const charts = [hipDof0Chart, hipDof1Chart];
    
    // Initialize start time if not set
    if (!window.hipChartStartTime) {
        window.hipChartStartTime = Date.now();
    }
    
    // Calculate time in seconds since start
    const now = Date.now();
    const timeSeconds = (now - window.hipChartStartTime) / 1000;
    
    // Throttle chart updates to ~10Hz (100ms interval)
    if (!window.lastHipChartUpdate) window.lastHipChartUpdate = 0;
    const shouldUpdateChart = (now - window.lastHipChartUpdate) > 100;
    
    charts.forEach((chartVar, dofIndex) => {
        if (chartVar && dataPoint.angles_deg[dofIndex] !== null) {
            // Add data point as {x, y} for linear x-axis
            chartVar.data.datasets[0].data.push({
                x: timeSeconds,
                y: dataPoint.angles_deg[dofIndex]
            });
            
            // Limit buffer size to 500 points
            if (chartVar.data.datasets[0].data.length > 500) {
                chartVar.data.datasets[0].data.shift();
            }
        }
    });
    
    // Only update chart rendering at throttled rate
    if (shouldUpdateChart) {
        charts.forEach(chartVar => {
            if (chartVar) chartVar.update('none');
        });
        window.lastHipChartUpdate = now;
    }
}

// Track if encoder streaming is via CAN
let encoderStreamingViaCan = false;

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
    
    // Stop based on mode (CAN or Serial)
    if (encoderStreamingViaCan) {
        // Stop CAN streaming
        $.ajax({
            url: '/can/encoder_stream/stop',
            type: 'POST'
        }).done(response => {
            appendStatusMessage(`⏹️ CAN encoder streaming stopped for ${currentEncoderJointType}`);
        }).fail(() => {
            appendStatusMessage(`⏹️ Encoder streaming stopped (error stopping CAN stream)`);
        });
        encoderStreamingViaCan = false;
    } else {
        // Stop serial test
    sendCommand('stop-test-encoder');
    appendStatusMessage(`⏹️ Encoder test stopped for ${currentEncoderJointType}`);
    }
    
    // Aggiorna UI
    updateEncoderTestUI(false, currentEncoderJointType);
    
    currentEncoderJointType = null;
}

/**
 * Toggle CAN Diagnostic panel visibility
 */
function toggleCanDiagnostic() {
    const panel = document.getElementById('canDiagnosticPanel');
    const icon = document.getElementById('canDiagnosticIcon');
    
    if (panel.style.display === 'none') {
        panel.style.display = 'block';
        icon.classList.remove('fa-plus-circle');
        icon.classList.add('fa-minus-circle');
    } else {
        panel.style.display = 'none';
        icon.classList.remove('fa-minus-circle');
        icon.classList.add('fa-plus-circle');
    }
}

/**
 * Runs CAN bus diagnostic test
 * Tests Motor CAN (J4) communication with motors
 */
function runCanDiagnostic() {
    const joint = $("#jointSelect").val();
    
    if (!joint) {
        appendStatusMessage("⚠️ Select a joint first");
        return;
    }
    
    appendStatusMessage(`🔍 Running CAN diagnostic for ${joint}...`);
    appendStatusMessage(`   Motor CAN (J4) only - Host CAN (J5) disabled`);
    
    sendCommand('can-diag');
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

// ============================================================================
// UI HELPER FUNCTIONS
// ============================================================================

/**
 * Toggles visibility of advanced movement parameters
 */
function toggleAdvancedParams() {
    const container = $("#advancedParamsContainer");
    const icon = $("#advancedParamsIcon");
    
    if (container.is(":visible")) {
        container.slideUp(200);
        icon.removeClass("fa-minus-circle").addClass("fa-plus-circle");
    } else {
        container.slideDown(200);
        icon.removeClass("fa-plus-circle").addClass("fa-minus-circle");
    }
}

// ============================================================================
// AUTO-MAPPING PROGRESS AND VISUALIZATION
// ============================================================================

let autoMappingActive = false;
let autoMappingGridPoints = [];
let autoMappingCurrentIndex = 0;
let autoMappingTotalPoints = 0;

/**
 * Updates auto-mapping progress UI and highlights current point
 */
function updateAutoMappingProgress(data) {
    const progressDiv = $("#autoMappingProgress");
    const gridInfoDiv = $("#autoMappingGridInfo");
    const counterSpan = $("#autoMappingCounter");
    const progressBar = $("#autoMappingProgressBar");
    const currentPointDiv = $("#autoMappingCurrentPoint");
    const gridDetails = $("#autoMappingGridDetails");
    
    // Check if mapping is complete or if data contains progress info
    const totalPoints = data.total_points || 0;
    const currentPoint = data.current_point || (data.data ? data.data.total_points : 0);
    
    // If we have mapping data, show grid info
    if (data.data && data.data.present_dofs && data.data.present_dofs.length > 0) {
        // Calculate grid dimensions from mapping data
        autoMappingTotalPoints = totalPoints;
        autoMappingGridPoints = calculateMappingGridPoints(data.data);
        
        // Show grid info
        gridInfoDiv.show();
        let gridHTML = '';
        data.data.present_dofs.forEach((dofIdx) => {
            const dofData = data.data[`dof_${dofIdx}`];
            if (dofData && dofData.joint_angles) {
                const angles = dofData.joint_angles;
                const minAngle = Math.min(...angles);
                const maxAngle = Math.max(...angles);
                const numPoints = angles.length;
                const step = numPoints > 1 ? ((maxAngle - minAngle) / (numPoints - 1)).toFixed(1) : 0;
                gridHTML += `<div>DOF ${dofIdx}: ${minAngle.toFixed(1)}° to ${maxAngle.toFixed(1)}° (${numPoints} points, step ~${step}°)</div>`;
            }
        });
        gridHTML += `<div class="font-semibold mt-1">Total: ${autoMappingTotalPoints} points</div>`;
        gridDetails.html(gridHTML);
        
        // Check if mapping is still in progress
        if (currentPoint < totalPoints && currentPoint > 0) {
            autoMappingActive = true;
            autoMappingCurrentIndex = currentPoint;
            
            // Show progress
            progressDiv.show();
            const percentage = (currentPoint / totalPoints * 100).toFixed(1);
            counterSpan.text(`${currentPoint} / ${totalPoints}`);
            progressBar.css('width', `${percentage}%`);
            
            // Show current point angles if available
            if (autoMappingGridPoints[currentPoint]) {
                const point = autoMappingGridPoints[currentPoint];
                currentPointDiv.text(`Current: DOF0=${point.dof0.toFixed(1)}°, DOF1=${point.dof1.toFixed(1)}°`);
                
                // Highlight current point on grid buttons
                highlightMappingPointOnGrid(point.dof0, point.dof1);
            }
        } else if (currentPoint >= totalPoints) {
            // Mapping complete
            autoMappingActive = false;
            progressDiv.hide();
            appendStatusMessage(`✅ Auto-mapping completed: ${totalPoints} points acquired`);
            
            // Remove highlights
            removeAllMappingHighlights();
        }
    } else if (currentPoint === 0 && totalPoints > 0) {
        // Mapping just started
        autoMappingActive = true;
        progressDiv.show();
        counterSpan.text(`0 / ${totalPoints}`);
        progressBar.css('width', '0%');
        currentPointDiv.text('Initializing...');
    }
}

/**
 * Calculates grid points from mapping data for visualization
 */
function calculateMappingGridPoints(mappingData) {
    const points = [];
    
    if (!mappingData || !mappingData.present_dofs) return points;
    
    const presentDofs = mappingData.present_dofs;
    
    // For 2-DOF joints (ANKLE), create grid of all combinations
    if (presentDofs.length === 2) {
        const dof0Data = mappingData[`dof_${presentDofs[0]}`];
        const dof1Data = mappingData[`dof_${presentDofs[1]}`];
        
        if (dof0Data && dof1Data) {
            dof0Data.joint_angles.forEach(angle0 => {
                dof1Data.joint_angles.forEach(angle1 => {
                    points.push({dof0: angle0, dof1: angle1});
                });
            });
        }
    }
    // For 1-DOF joints (KNEE), just use DOF 0
    else if (presentDofs.length === 1) {
        const dof0Data = mappingData[`dof_${presentDofs[0]}`];
        if (dof0Data) {
            dof0Data.joint_angles.forEach(angle0 => {
                points.push({dof0: angle0, dof1: 0});
            });
        }
    }
    
    return points;
}

/**
 * Highlights a specific point on the grid buttons during auto-mapping
 */
function highlightMappingPointOnGrid(dof0, dof1) {
    // Remove previous highlights
    $(".mapping-highlight").removeClass("mapping-highlight");
    
    // Find button matching these angles (with tolerance of ±0.5°)
    $("#smartQuickButtons button").each(function() {
        const btn = $(this);
        const title = btn.attr("title");
        
        if (title) {
            // Parse title like "DOF 0: 25°, DOF 1: -15°"
            const match = title.match(/DOF 0: ([-\d.]+)°, DOF 1: ([-\d.]+)°/);
            if (match) {
                const btnDof0 = parseFloat(match[1]);
                const btnDof1 = parseFloat(match[2]);
                
                // Check if angles match (with tolerance)
                if (Math.abs(btnDof0 - dof0) < 0.5 && Math.abs(btnDof1 - dof1) < 0.5) {
                    btn.addClass("mapping-highlight");
                    
                    // Scroll into view
                    btn[0].scrollIntoView({ behavior: 'smooth', block: 'nearest' });
                }
            }
        }
    });
}

/**
 * Removes all mapping highlights from grid buttons
 */
function removeAllMappingHighlights() {
    $(".mapping-highlight").removeClass("mapping-highlight");
}

/**
 * Fetches joint configuration from joint_config.json
 */
function fetchJointConfig() {
    $.ajax({
        url: '/joint_config',
        method: 'GET',
        success: function(response) {
            if (response.status === 'success' && response.config) {
                jointConfigData = response.config;
                console.log('Joint configuration loaded:', jointConfigData);
                updateCanMotionJoint();
                
                // Initialize sinusoid parameters for the currently selected joint
                const initialJoint = $("#jointSelect").val();
                updateSinusoidParamsForJoint(initialJoint);
                
                // Initialize sinusoid stats with default slider value (100 pts/s)
                updateSinusoidStats($("#waypointDensity").val() || 100);
                
                // Show expected mapping grid for initially selected joint
                showExpectedMappingGrid(initialJoint);
            } else {
                console.error('Failed to load joint config:', response);
            }
        },
        error: function(xhr, status, error) {
            console.error('Error fetching joint config:', error);
            appendStatusMessage('⚠️ Could not load joint configuration');
        }
    });
}

/**
 * Shows expected mapping grid for a given joint BEFORE mapping starts
 * Based on joint configuration from joint_config.json
 */
function showExpectedMappingGrid(jointName) {
    if (!jointConfigData || !jointConfigData.joints) {
        return;
    }
    
    // Normalize joint name (e.g., "ANKLE_LEFT" -> "ankle_left")
    const normalizedName = jointName.toLowerCase();
    const jointConfig = jointConfigData.joints[normalizedName];
    
    if (!jointConfig) {
        console.warn(`No config found for joint: ${jointName}`);
        return;
    }
    
    const gridInfoDiv = $("#autoMappingGridInfo");
    const gridDetails = $("#autoMappingGridDetails");
    
    // Build grid info HTML
    let gridHTML = '';
    let totalPoints = 1;
    
    jointConfig.dofs.forEach((dof, idx) => {
        // Use auto_mapping parameters from firmware config (more accurate than physical limits)
        const minAngle = dof.auto_mapping_min_angle !== undefined ? dof.auto_mapping_min_angle : dof.min_angle;
        const maxAngle = dof.auto_mapping_max_angle !== undefined ? dof.auto_mapping_max_angle : dof.max_angle;
        const step = dof.auto_mapping_step !== undefined ? dof.auto_mapping_step : 5.0;
        
        const range = maxAngle - minAngle;
        const numPoints = Math.floor(range / step) + 1;
        
        gridHTML += `<div>DOF ${idx}: ${minAngle.toFixed(1)}° to ${maxAngle.toFixed(1)}° (${numPoints} points, step ${step.toFixed(1)}°)</div>`;
        
        totalPoints *= numPoints;
    });
    
    gridHTML += `<div class="font-semibold mt-1">Expected total: ${totalPoints} points</div>`;
    
    gridDetails.html(gridHTML);
    gridInfoDiv.show();
    
    // Store for later use
    autoMappingTotalPoints = totalPoints;
}

// ============================================================================
// MOVEMENT SEQUENCE BUILDER FUNCTIONS
// ============================================================================

/**
 * Updates visibility of sequence builder container based on joint type and toggle state
 * Only visible for ANKLE and KNEE joints AND when sequence mode toggle is active
 */
function updateSequenceBuilderVisibility(joint) {
    const container = $("#sequenceBuilderContainer");
    const isSequenceModeActive = $("#sequenceModeToggle").is(":checked");
    
    // Show only if: supported joint (ANKLE/KNEE) AND sequence mode toggle is ON
    if (joint && (joint.includes('ANKLE') || joint.includes('KNEE')) && isSequenceModeActive) {
        container.show();
    } else {
        container.hide();
        // Clear sequence when switching to non-supported joint
        if (!joint || (!joint.includes('ANKLE') && !joint.includes('KNEE'))) {
            if (movementSequence.length > 0) {
                clearSequence(true); // Silent clear
            }
        }
    }
}

/**
 * Adds a step to the movement sequence
 */
function addStepToSequence(dof0, dof1) {
    movementSequence.push({
        type: "move",
        dof0: dof0,
        dof1: dof1
    });
    
    renderSequenceList();
    
    // Show sequence builder if hidden
    $("#sequenceBuilderContainer").show();
}

/**
 * Adds a pause step to the movement sequence
 */
function addPauseToSequence() {
    const duration = parseFloat($("#pauseDuration").val()) || 1.0;
    
    movementSequence.push({
        type: "pause",
        duration: duration
    });
    
    renderSequenceList();
    appendStatusMessage(`⏸️ Added pause: ${duration}s`);
    
    // Show sequence builder if hidden
    $("#sequenceBuilderContainer").show();
}

/**
 * Removes a specific step from the sequence
 */
function removeStep(index) {
    if (index >= 0 && index < movementSequence.length) {
        movementSequence.splice(index, 1);
        renderSequenceList();
        appendStatusMessage(`🗑️ Removed step ${index + 1}`);
    }
}

/**
 * Removes the last step from the sequence
 */
function removeLastStep() {
    if (movementSequence.length === 0) {
        appendStatusMessage("⚠️ Sequence is already empty");
        return;
    }
    
    movementSequence.pop();
    renderSequenceList();
    appendStatusMessage(`⬅️ Removed last step (${movementSequence.length} steps remaining)`);
}

/**
 * Clears the entire sequence
 */
function clearSequence(silent = false) {
    if (!silent && movementSequence.length > 3) {
        if (!confirm(`Clear all ${movementSequence.length} steps from sequence?`)) {
            return;
        }
    }
    
    movementSequence = [];
    sequenceExecutionData = [];
    renderSequenceList();
    
    // Hide export button
    $("#exportSequenceBtn").hide();
    
    if (!silent) {
        appendStatusMessage("🗑️ Sequence cleared");
    }
}

/**
 * Renders the sequence list UI with all steps
 */
function renderSequenceList() {
    const container = $("#sequenceList");
    
    if (movementSequence.length === 0) {
        container.html('<div class="sequence-empty-state">Click grid buttons to add steps to sequence</div>');
        return;
    }
    
    let html = '';
    movementSequence.forEach((step, index) => {
        if (step.type === "pause") {
            // Render pause step
            html += `
                <div class="sequence-step pause" data-step-index="${index}">
                    <span class="sequence-step-number">Step ${index + 1}</span>
                    <span class="sequence-step-angles"><i class="fas fa-pause-circle mr-1"></i>Pause: ${step.duration}s</span>
                    <button class="sequence-step-remove" onclick="removeStep(${index})" title="Remove this step">×</button>
                </div>
            `;
        } else {
            // Render movement step
            html += `
                <div class="sequence-step" data-step-index="${index}">
                    <span class="sequence-step-number">Step ${index + 1}</span>
                    <span class="sequence-step-angles"><i class="fas fa-arrows-alt mr-1"></i>DOF0: ${step.dof0}°, DOF1: ${step.dof1}°</span>
                    <button class="sequence-step-remove" onclick="removeStep(${index})" title="Remove this step">×</button>
                </div>
            `;
        }
    });
    
    container.html(html);
}

/**
 * Highlights a specific step during playback
 */
function highlightSequenceStep(index) {
    // Remove current highlight from all steps
    $(".sequence-step").removeClass("current");
    
    // Add highlight to current step
    if (index >= 0 && index < movementSequence.length) {
        $(`.sequence-step[data-step-index="${index}"]`).addClass("current");
        
        // Scroll into view if needed
        const stepElement = $(`.sequence-step[data-step-index="${index}"]`)[0];
        if (stepElement) {
            stepElement.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
        }
    }
}

/**
 * Updates playback control buttons state
 */
function updatePlaybackControls() {
    if (isSequencePlaying) {
        $("#playSequenceBtn").prop("disabled", true).addClass("opacity-50");
        $("#stopSequenceBtn").prop("disabled", false).removeClass("opacity-50");
    } else {
        $("#playSequenceBtn").prop("disabled", false).removeClass("opacity-50");
        $("#stopSequenceBtn").prop("disabled", true).addClass("opacity-50");
    }
}

/**
 * Shows the export data button after playback
 */
function showExportButton() {
    if (sequenceExecutionData.length > 0) {
        $("#exportSequenceBtn").show();
    }
}

/**
 * Helper function to sleep for a specified duration (in ms)
 */
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

/**
 * Async version of sendMultiDofMove that waits for movement completion
 * This uses the backend acknowledgment system that waits for RSP:MOVE_COMPLETE
 */
function sendMultiDofMoveAsync() {
    return new Promise((resolve, reject) => {
        const joint = $("#jointSelect").val();
        const angle0 = parseFloat($("#multiDofAngle0").val()) || 0;
        const angle1 = parseFloat($("#multiDofAngle1").val()) || 0;
        const mask = parseInt($("#multiDofMask").val()) || 3;
        const sync = parseInt($("#multiDofSync").val()) ?? 1;
        const speed = parseFloat($("#multiDofSpeed").val()) || 0.5;
        const accel = parseFloat($("#multiDofAccel").val()) || 2.0;
        const path = parseInt($("#multiDofPath").val()) ?? 1;
        
        // Validation
        if (!joint) {
            reject(new Error("No joint selected"));
            return;
        }
        
        const assignedPort = jointPortMapping[joint];
        if (!assignedPort) {
            appendStatusMessage(`⚠️ Associate a serial port to ${joint} before sending command.`);
            reject(new Error("No serial port assigned"));
            return;
        }
        
        // Build command data
        const multiDofData = {
            joint: joint,
            angle0: angle0,
            angle1: angle1,
            angle2: 0,
            mask: mask,
            sync: sync,
            speed: speed,
            accel: accel,
            path: path
        };
        
        const data = { cmd: 'move-multi-dof', joint: joint, dof: 'ALL', ...multiDofData };
        
        // Send command and WAIT for completion (backend waits for RSP:MOVE_COMPLETE)
        $.ajax({
            url: "/command",
            method: "POST",
            data: JSON.stringify(data),
            contentType: "application/json; charset=utf-8",
            dataType: "json",
            timeout: 30000, // 30 second timeout
            success: function(response) {
                appendStatusMessage(response.message);
                resolve(response);
            },
            error: function(xhr, status, error) {
                const errorMsg = `Error sending move command: ${error}`;
                appendStatusMessage(`❌ ${errorMsg}`);
                reject(new Error(errorMsg));
            }
        });
    });
}

/**
 * Main playback function - executes the movement sequence
 */
async function playSequence() {
    if (movementSequence.length === 0) {
        alert("Sequence is empty. Add steps first.");
        return;
    }
    
    const loopEnabled = $("#sequenceLoopToggle").is(":checked");
    isSequencePlaying = true;
    sequenceExecutionData = []; // Reset data collection
    updatePlaybackControls();
    
    // Start sequence data collection on backend
    try {
        await $.ajax({
            url: "/sequence/start",
            method: "POST",
            contentType: "application/json"
        });
        appendStatusMessage(`📊 Backend data collection started`);
    } catch (error) {
        console.error("Error starting sequence data collection:", error);
    }
    
    appendStatusMessage(`▶️ Starting sequence playback (${movementSequence.length} steps${loopEnabled ? ', loop enabled' : ''})`);
    
    let loopCount = 0;
    
    do {
        if (loopEnabled) {
            loopCount++;
            appendStatusMessage(`🔄 Loop iteration ${loopCount}`);
        }
        
        for (let i = 0; i < movementSequence.length; i++) {
            if (!isSequencePlaying) {
                appendStatusMessage("⏹️ Sequence stopped by user");
                break;
            }
            
            const step = movementSequence[i];
            const startTime = Date.now();
            
            if (step.type === "pause") {
                // Handle pause step
                appendStatusMessage(`  Step ${i + 1}/${movementSequence.length}: ⏸️ Pause ${step.duration}s`);
                
                // Highlight current step AFTER logging, BEFORE execution
                highlightSequenceStep(i);
                
                await sleep(step.duration * 1000); // Convert seconds to milliseconds
                
                // Record execution (optional, for tracking)
                const executionRecord = {
                    stepIndex: i,
                    loopIteration: loopCount,
                    type: "pause",
                    duration: step.duration,
                    startTimestamp: startTime,
                    endTimestamp: Date.now()
                };
                sequenceExecutionData.push(executionRecord);
            } else {
                // Handle movement step
                // Set angles in UI
                $("#multiDofAngle0").val(step.dof0);
                $("#multiDofAngle1").val(step.dof1);
                updateMultiDofCommandPreview();
                
                // Collect execution data
                const executionRecord = {
                    stepIndex: i,
                    loopIteration: loopCount,
                    type: "move",
                    targetDof0: step.dof0,
                    targetDof1: step.dof1,
                    startTimestamp: startTime,
                    encoderSamples: [] // Will be populated by socket listener
                };
                
                sequenceExecutionData.push(executionRecord);
                
                appendStatusMessage(`  Step ${i + 1}/${movementSequence.length}: DOF0=${step.dof0}°, DOF1=${step.dof1}°`);
                
                // Highlight current step AFTER logging, BEFORE sending command
                highlightSequenceStep(i);
                
                // Execute movement - now waits for acknowledgment from firmware
                await sendMultiDofMoveAsync();
                
                executionRecord.endTimestamp = Date.now();
                executionRecord.actualDuration = executionRecord.endTimestamp - executionRecord.startTimestamp;
            }
        }
    } while (loopEnabled && isSequencePlaying);
    
    isSequencePlaying = false;
    highlightSequenceStep(-1); // Remove all highlights
    updatePlaybackControls();
    
    // Stop sequence data collection on backend
    try {
        const response = await $.ajax({
            url: "/sequence/stop",
            method: "POST",
            contentType: "application/json"
        });
        appendStatusMessage(`📊 Backend data collection stopped - ${response.steps_collected} steps collected`);
    } catch (error) {
        console.error("Error stopping sequence data collection:", error);
    }
    
    showExportButton();
    
    appendStatusMessage(`✅ Sequence playback completed (${sequenceExecutionData.length} executions recorded)`);
}

/**
 * Stops the sequence playback
 */
function stopSequence() {
    if (isSequencePlaying) {
        isSequencePlaying = false;
        appendStatusMessage("⏹️ Stopping sequence playback...");
        // The playback loop will stop at next iteration check
    }
}

/**
 * Exports sequence movement data as CSV file
 */
async function exportSequenceData() {
    try {
        appendStatusMessage(`📥 Fetching sequence movement data from backend...`);
        
        // Get accumulated movement data from backend
        const response = await $.ajax({
            url: "/sequence/data",
            method: "GET"
        });
        
        if (response.status !== "success" || !response.data || response.data.length === 0) {
            alert("No movement data available. The sequence may not have been executed or data collection failed.");
            return;
        }
        
        appendStatusMessage(`📊 Processing ${response.steps} steps of movement data...`);
        
        // Build CSV
        const csvLines = [];
        
        // CSV Header
        csvLines.push([
            "Step",
            "Joint",
            "DOF",
            "Sample_Index",
            "Joint_Target_deg",
            "Joint_Actual_deg",
            "Motor_Agonist_Current_deg",
            "Motor_Antagonist_Current_deg",
            "Motor_Agonist_Ref_deg",
            "Motor_Antagonist_Ref_deg",
            "Torque_Agonist",
            "Torque_Antagonist"
        ].join(","));
        
        // Process each step
        for (const stepData of response.data) {
            const stepIndex = stepData.step_index;
            const jointName = stepData.joint_name;
            const dofData = stepData.dof_data;
            
            // Process each DOF
            for (const [dofIndex, dofArrays] of Object.entries(dofData)) {
                const sampleCount = dofArrays.joint_targets.length;
                
                // Process each sample
                for (let i = 0; i < sampleCount; i++) {
                    const row = [
                        stepIndex,
                        jointName,
                        dofIndex,
                        i,
                        dofArrays.joint_targets[i].toFixed(4),
                        dofArrays.joint_angles[i].toFixed(4),
                        dofArrays.motor_angles.agonist_current[i].toFixed(4),
                        dofArrays.motor_angles.antagonist_current[i].toFixed(4),
                        dofArrays.motor_angles.agonist_next[i].toFixed(4),
                        dofArrays.motor_angles.antagonist_next[i].toFixed(4),
                        dofArrays.motor_torques.agonist[i].toFixed(2),
                        dofArrays.motor_torques.antagonist[i].toFixed(2)
                    ];
                    csvLines.push(row.join(","));
                }
            }
        }
        
        // Create CSV file
        const csvContent = csvLines.join("\n");
        const blob = new Blob([csvContent], {type: 'text/csv;charset=utf-8;'});
        const url = URL.createObjectURL(blob);
        const link = document.createElement('a');
        link.href = url;
        
        const timestamp = new Date().toISOString().replace(/[:.]/g, '-');
        const joint = $("#jointSelect").val();
        link.download = `sequence_movement_${joint}_${timestamp}.csv`;
        
        link.click();
        URL.revokeObjectURL(url);
        
        const totalSamples = csvLines.length - 1; // Exclude header
        appendStatusMessage(`💾 Exported ${totalSamples} movement samples from ${response.steps} steps to CSV`);
        
    } catch (error) {
        console.error("Error exporting sequence data:", error);
        appendStatusMessage(`❌ Error exporting sequence data: ${error.message || error}`);
        alert("Failed to export sequence data. Check console for details.");
    }
}

