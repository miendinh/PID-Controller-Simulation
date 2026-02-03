// Physics Constants
const GRAVITY = 9.81;
const DT = 0.02; // Time step (s)

class PIDController {
    constructor(kp, ki, kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.prevError = 0;
        this.integral = 0;
    }

    update(setpoint, measuredValue) {
        const error = setpoint - measuredValue;
        this.integral += error * DT;
        const derivative = (error - this.prevError) / DT;

        this.pTerm = this.kp * error;
        this.iTerm = this.ki * this.integral;
        this.dTerm = this.kd * derivative;

        const output = this.pTerm + this.iTerm + this.dTerm;
        this.prevError = error;
        return output;
    }

    reset() {
        this.prevError = 0;
        this.integral = 0;
    }
}

class InvertedPendulum {
    constructor() {
        // State variables
        this.x = 0;           // Cart position (m)
        this.dx = 0;          // Cart velocity (m/s)
        this.theta = 0.2;     // Pole angle (rad) - start with small tilt
        this.dtheta = 0;      // Pole angular velocity (rad/s)

        // Physical parameters
        this.M = 1.0;         // Mass of cart (kg)
        this.m = 0.5;         // Mass of pole (kg)
        this.L = 1.0;         // Length of pole (m)

        // Limits
        this.xLimit = 2.5;    // Track limit (m)
    }

    update(force) {
        // Equations of motion for Inverted Pendulum on a Cart
        // Source: http://ctms.engin.umich.edu/CTMS/index.php?example=InvertedPendulum&section=SystemModeling

        const sinTheta = Math.sin(this.theta);
        const cosTheta = Math.cos(this.theta);

        // Denominator common to both equations
        const temp = this.m * this.L * this.dtheta * this.dtheta * sinTheta;
        const denominator = (this.M + this.m) - this.m * cosTheta * cosTheta;

        // Acceleration of theta (angular acceleration)
        const ddtheta = ((this.M + this.m) * GRAVITY * sinTheta - cosTheta * (force + temp)) / (this.L * denominator);

        // Acceleration of x (linear acceleration)
        const ddx = (force + temp + this.m * GRAVITY * sinTheta * cosTheta) / denominator; // Simplified

        // More accurate x acceleration derived from solving systems
        // F = (M+m)ddx - mL(ddtheta)cosTheta + mL(dtheta^2)sinTheta
        // But let's stick to the coupled equations above or a simpler approximate if needed.
        // Actually, let's use the explicit forms:

        // Update velocities
        this.dx += ddx * DT;
        this.dtheta += ddtheta * DT;

        // Update positions
        this.x += this.dx * DT;
        this.theta += this.dtheta * DT;

        // REMOVED ARTIFICIAL FRICTION needed to demonstrate Kd clearly!
        // this.dx *= 0.99; 
        // this.dtheta *= 0.99;
    }

    applyExternalForce(f) {
        this.dx += f * 5 * DT; // Instant push effect
    }

    // Add method to apply constant wind
    applyWind(f) {
        this.dx += f * DT;
    }

    reset() {
        this.x = 0;
        this.dx = 0;
        this.theta = Math.random() * 0.2 - 0.1; // Random small start angle
        this.dtheta = 0;
    }
}

// Canvas & Simulation Setup
const canvas = document.getElementById('simCanvas');
const angleDisplay = document.getElementById('angleValue');
const dthetaDisplay = document.getElementById('dthetaValue');
const xDisplay = document.getElementById('xValue');
const dxDisplay = document.getElementById('dxValue');
const motorDisplay = document.getElementById('motorValue');

// --- THREE.JS SETUP ---
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f172a); // Match bg color

// Camera
const camera = new THREE.PerspectiveCamera(45, canvas.clientWidth / 500, 0.1, 100);
camera.position.set(0, 2, 6);
camera.lookAt(0, 1, 0);

// Renderer
const renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true, alpha: true });
renderer.setSize(canvas.clientWidth, 500);
renderer.shadowMap.enabled = true;

// Lights
const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 10, 7);
dirLight.castShadow = true;
scene.add(dirLight);

// --- 3D OBJECTS ---

// Ground
const gridHelper = new THREE.GridHelper(50, 50, 0x475569, 0x1e293b);
scene.add(gridHelper);

// Robot Group
const robotGroup = new THREE.Group();
scene.add(robotGroup);

// Cart
const cartGeometry = new THREE.BoxGeometry(0.8, 0.3, 0.4); // Made longer (0.6 -> 0.8)
const cartMaterial = new THREE.MeshPhongMaterial({ color: 0x3b82f6 });
const cartMesh = new THREE.Mesh(cartGeometry, cartMaterial);
cartMesh.position.y = 0.15 + 0.15; // Enable wheels height
cartMesh.castShadow = true;
robotGroup.add(cartMesh);

// Wheels
const wheelGeometry = new THREE.CylinderGeometry(0.15, 0.15, 0.1, 16);
const wheelMaterial = new THREE.MeshPhongMaterial({ color: 0x1e293b });

const wheel1 = new THREE.Mesh(wheelGeometry, wheelMaterial);
wheel1.rotation.x = Math.PI / 2; // Orient cylinder to face Z
wheel1.position.set(-0.3, 0.15, 0.25); // Adjusted X offset (-0.2 -> -0.3)
robotGroup.add(wheel1);

const wheel2 = new THREE.Mesh(wheelGeometry, wheelMaterial);
wheel2.rotation.x = Math.PI / 2;
wheel2.position.set(0.3, 0.15, 0.25); // Adjusted X offset (0.2 -> 0.3)
robotGroup.add(wheel2);

const wheel3 = new THREE.Mesh(wheelGeometry, wheelMaterial);
wheel3.rotation.x = Math.PI / 2;
wheel3.position.set(-0.3, 0.15, -0.25); // Adjusted X offset
robotGroup.add(wheel3);

const wheel4 = new THREE.Mesh(wheelGeometry, wheelMaterial);
wheel4.rotation.x = Math.PI / 2;
wheel4.position.set(0.3, 0.15, -0.25); // Adjusted X offset
robotGroup.add(wheel4);

// Spoke helper
function addSpoke(wheel) {
    const spokeGeom = new THREE.BoxGeometry(0.25, 0.05, 0.02);
    const spokeMat = new THREE.MeshBasicMaterial({ color: 0x94a3b8 });
    const spoke = new THREE.Mesh(spokeGeom, spokeMat);
    spoke.position.z = 0.06; // Slightly in front of face
    wheel.add(spoke);

    const spoke2 = spoke.clone();
    spoke2.rotation.y = Math.PI / 2; // BoxGeometry local axis is Z? No.
    // Box 0.25 wide (X). 
    // If cylinder axis is Y. Spoke should be in XZ plane.
    // wait.
    // If I use rotation.x on Mesh to flip cylinder.
    // Then local Y is Z-world? No.
    // X -> X.
    // Y -> Z.
    // Z -> -Y.
    // Spoke position Z=0.06 is in local Z. So relative to world it's -Y?
    // Wait.
    // Let's keep it simple.
    // Default Cylinder Axis is Y.
    // Top cap is at y = h/2.
    // We want spoke on top cap.
    // Position y = h/2 + epsilon.
    // Spoke lies in XZ plane.
    wheel.add(spoke);

    // BUT spoke code used Z position.
    // Previous geometry was rotated so Z was the face.
    // Now Geometry is default (Y axis).
    // So Face is Y.
    spoke.position.set(0, 0.06, 0); // On Y face
    spoke.rotation.x = Math.PI / 2; // Flat on cap?

}
[wheel1, wheel2, wheel3, wheel4].forEach(addSpoke);

// Pole Pivot/Joint
const pivotGroup = new THREE.Group();
pivotGroup.position.y = 0.3 + 0.15; // On top of cart
robotGroup.add(pivotGroup);

// Pole
const poleGeometry = new THREE.CylinderGeometry(0.04, 0.04, 1.0); // Length 1.0 (matches robot.L)
const poleMaterial = new THREE.MeshPhongMaterial({ color: 0x22d3ee });
const poleMesh = new THREE.Mesh(poleGeometry, poleMaterial);
poleMesh.position.y = 0.5; // Half height up
poleMesh.castShadow = true;
pivotGroup.add(poleMesh);

// Mass
const massGeometry = new THREE.SphereGeometry(0.12, 16, 16);
const massMaterial = new THREE.MeshPhongMaterial({ color: 0xef4444 });
const massMesh = new THREE.Mesh(massGeometry, massMaterial);
massMesh.position.y = 1.0; // Top of pole
massMesh.castShadow = true;
pivotGroup.add(massMesh);


function resizeCanvas() {
    const width = canvas.parentElement.clientWidth;
    const height = 500;
    renderer.setSize(width, height);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
}
window.addEventListener('resize', resizeCanvas);
// resizeCanvas(); // Called by renderer init usually


const robot = new InvertedPendulum();
const pid = new PIDController(120, 0.5, 45); // Optimised defaults

// UI Controls
const kpInput = document.getElementById('kp');
const kiInput = document.getElementById('ki');
const kdInput = document.getElementById('kd');
const kpVal = document.getElementById('kpVal');
const kiVal = document.getElementById('kiVal');
const kdVal = document.getElementById('kdVal');
const windInput = document.getElementById('wind');
const windVal = document.getElementById('windVal');

function updatePID() {
    pid.kp = parseFloat(kpInput.value);
    pid.ki = parseFloat(kiInput.value);
    pid.kd = parseFloat(kdInput.value);
    kpVal.textContent = pid.kp.toFixed(1);
    kiVal.textContent = pid.ki.toFixed(2);
    kdVal.textContent = pid.kd.toFixed(1);

    const wind = parseFloat(windInput.value);
    windVal.textContent = wind.toFixed(1) + " (Test Ki)";
    // pid.reset(); // Don't reset on param change, allows live tuning better
}

windInput.addEventListener('input', updatePID);
kpInput.addEventListener('input', updatePID);
kiInput.addEventListener('input', updatePID);
kdInput.addEventListener('input', updatePID);

document.getElementById('resetBtn').addEventListener('click', () => {
    robot.reset();
    pid.reset();
});

document.getElementById('pushLeftBtn').addEventListener('click', () => {
    robot.dtheta += 3.0; // Stronger kick to the pole
    robot.dx += 2.0; // Also shove the cart
});

document.getElementById('pushRightBtn').addEventListener('click', () => {
    robot.dtheta -= 3.0; // Stronger kick to the pole
    robot.dx -= 2.0; // Also shove the cart
});

// Simulation Loop


function loop() {
    // 1. Calculate Control Output (PID)
    // Target is 0 (vertical). Input is current theta.
    // PID output is Force applied to Cart.

    // We need Positive Force to move Cart Right when Theta is Positive (leaning Right) to balance it.
    // PID Error calculation:
    // If we use (Target=0, Current=theta), Error = -theta. Kp*(-theta) = Negative force -> Wrong direction!
    // We want Error = theta. So use (Target=theta, Current=0) or just Negate the result.
    // Let's use (robot.theta, 0) so error = theta - 0 = positive when leaning right.
    const controlForce = pid.update(robot.theta, 0);

    // Limit force to realistic motor values
    const maxForce = 500;
    let clampedForce = Math.max(-maxForce, Math.min(maxForce, controlForce));

    // 2. Update Physics
    // If robot falls too far, create a 'crashed' state or just let it spin?
    // Let's stop simulation if it falls over (> 60 degrees) for realism, or just let it swing.
    // For PID tuning, usually we want to see it fail.

    if (Math.abs(robot.theta) < Math.PI / 2) {
        robot.update(clampedForce);
        robot.applyWind(parseFloat(windInput.value) * 5.0); // Apply continuous wind
    } else {
        // Crashed - dampen velocities
        robot.dx *= 0.9;
        robot.dtheta *= 0.9;
        // Gravity still applies though?
        // Let's just keep updating physics but without motor force to simulate 'engine cutoff'
        robot.update(0);
    }

    // Boundary check for track
    if (robot.x > 3 && robot.dx > 0) robot.dx = -robot.dx * 0.5;
    if (robot.x < -3 && robot.dx < 0) robot.dx = -robot.dx * 0.5;

    // 3. Update UI
    angleDisplay.textContent = (robot.theta * 180 / Math.PI).toFixed(2);
    dthetaDisplay.textContent = robot.dtheta.toFixed(2);
    xDisplay.textContent = robot.x.toFixed(2);
    dxDisplay.textContent = robot.dx.toFixed(2);
    motorDisplay.textContent = clampedForce.toFixed(2);

    document.getElementById('pTerm').textContent = pid.pTerm ? pid.pTerm.toFixed(1) : "0.0";
    document.getElementById('iTerm').textContent = pid.iTerm ? pid.iTerm.toFixed(1) : "0.0";
    document.getElementById('dTerm').textContent = pid.dTerm ? pid.dTerm.toFixed(1) : "0.0";

    // 4. Render
    // 4. Update 3D Scene

    // Sync Robot Position
    robotGroup.position.x = robot.x;

    // Sync Pole Angle
    // In Three.js, rotation Z is axis sticking out of screen.
    // Positive theta (right tip) -> Negative rotation Z? 
    // Let's check: x is right, y is up.
    // If pole leans right (x+), simple rotation around Z.
    // Math.sin(theta) * L -> x.
    pivotGroup.rotation.z = -robot.theta; // Negative because Z-rot + is CCW usually? Let's verify visually.

    // Wheels rotation
    const realWheelRadius = 0.15;
    const wheelRot = -robot.x / realWheelRadius; // Rolling
    wheel1.rotation.y = wheelRot; // Cylinder laying on Z, means local Y is rotation axis? No.
    // CylinderGeometry defaults to upright (connecting top/bottom on Y).
    // I rotated them Z=PI/2, so now they lie along X axis.
    // Wait, cylinder along X?
    // Geometry(radius, radius, height). 
    // Default Y axis. Rot Z 90 -> X axis.
    // To roll forward (along X world), it should rot around Z world?
    // Actually, let's simplify.
    // If cylinder axis is Z, rotation is Z.
    // I made them lie on Z? No, I want axle along Z.
    // "wheel1.rotation.z = Math.PI / 2" makes cylinder lie along X. Wrong.
    // I want cylinder axis along Z. So rotate X 90?
    // Let's re-orient wheels in setup if this looks wrong.

    // Correction for wheels:
    // Cylinder geometry default (Axis Y).
    // Mesh rotated X 90 -> Axis points to Camera (Z).
    // Rolling along X => Rotation around world Z.
    // Since Mesh Axis is Y (pointing to World Z), we rotate around Local Y.
    // Direction:
    // World Z points out.
    // Rolling Right (X+) -> Clockwise (thumb in).
    // Y axis points out.
    // Rotation around Y should be negative for CW.
    const wheelAngle = -robot.x / realWheelRadius;
    wheel1.rotation.y = wheelAngle;
    wheel2.rotation.y = wheelAngle;
    wheel3.rotation.y = wheelAngle;
    wheel4.rotation.y = wheelAngle;

    // Camera Tuning - Smooth follow
    // camera.position.x = robot.x; // Strict follow
    const targetCamX = robot.x;
    camera.position.x += (targetCamX - camera.position.x) * 0.1; // Smooth lerp
    camera.lookAt(camera.position.x, 1, 0);

    renderer.render(scene, camera);

    requestAnimationFrame(loop);
}

// Start
updatePID(); // initialize pid values
loop();
