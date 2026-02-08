// ================================================================
// ROBOT KINEMATICS TEST SUITE
// ================================================================

// ================================================================
//  SECTION 1: MATH UTILITIES
// ================================================================

const DEG = 180 / Math.PI;
const RAD = Math.PI / 180;

function dhTransform(theta, d, a, alpha) {
  const ct = Math.cos(theta), st = Math.sin(theta);
  const ca = Math.cos(alpha), sa = Math.sin(alpha);
  return [
    ct, -st * ca,  st * sa, a * ct,
    st,  ct * ca, -ct * sa, a * st,
    0,   sa,       ca,      d,
    0,   0,        0,       1
  ];
}

function mat4Mul(A, B) {
  const R = new Array(16);
  for (let r = 0; r < 4; r++)
    for (let c = 0; c < 4; c++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += A[r * 4 + k] * B[k * 4 + c];
      R[r * 4 + c] = s;
    }
  return R;
}

function mat4Identity() {
  return [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
}

function mat4Pos(M)  { return [M[3], M[7], M[11]]; }
function mat4AxisZ(M){ return [M[2], M[6], M[10]]; }

function v3Sub(a, b) { return [a[0]-b[0], a[1]-b[1], a[2]-b[2]]; }
function v3Add(a, b) { return [a[0]+b[0], a[1]+b[1], a[2]+b[2]]; }
function v3Scale(v, s){ return [v[0]*s, v[1]*s, v[2]*s]; }
function v3Cross(a, b){
  return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
}
function v3Dot(a, b){ return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
function v3Len(v){ return Math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }

function solveLinear(A, b, n) {
  const aug = [];
  for (let i = 0; i < n; i++) {
    aug[i] = new Float64Array(n + 1);
    for (let j = 0; j < n; j++) aug[i][j] = A[i * n + j];
    aug[i][n] = b[i];
  }
  for (let col = 0; col < n; col++) {
    let mx = col, mv = Math.abs(aug[col][col]);
    for (let r = col + 1; r < n; r++) {
      const v = Math.abs(aug[r][col]);
      if (v > mv) { mv = v; mx = r; }
    }
    [aug[col], aug[mx]] = [aug[mx], aug[col]];
    const p = aug[col][col];
    if (Math.abs(p) < 1e-12) continue;
    for (let r = col + 1; r < n; r++) {
      const f = aug[r][col] / p;
      for (let j = col; j <= n; j++) aug[r][j] -= f * aug[col][j];
    }
  }
  const x = new Float64Array(n);
  for (let i = n - 1; i >= 0; i--) {
    x[i] = aug[i][n];
    for (let j = i + 1; j < n; j++) x[i] -= aug[i][j] * x[j];
    if (Math.abs(aug[i][i]) > 1e-12) x[i] /= aug[i][i]; else x[i] = 0;
  }
  return x;
}

// ================================================================
//  SECTION 2: ROBOT KINEMATICS
// ================================================================

const PRESETS = {
  '6dof': {
    name: '6-DOF Industrial',
    dh: [ // [d, a, alpha]  theta is variable
      [0.40, 0.00,  Math.PI/2],
      [0.00, 0.80,  0],
      [0.00, 0.55,  0],
      [0.00, 0.00,  Math.PI/2],
      [0.38, 0.00, -Math.PI/2],
      [0.18, 0.00,  0]
    ],
    limits: [[-180,180],[-135,135],[-150,150],[-180,180],[-120,120],[-180,180]]
  },
  'ur5': {
    name: 'UR5-like',
    dh: [
      [0.089, 0.00,   Math.PI/2],
      [0.00,  -0.425, 0],
      [0.00,  -0.392, 0],
      [0.109, 0.00,   Math.PI/2],
      [0.095, 0.00,  -Math.PI/2],
      [0.082, 0.00,   0]
    ],
    limits: [[-360,360],[-360,360],[-360,360],[-360,360],[-360,360],[-360,360]]
  },
  'scara': {
    name: '4-DOF SCARA',
    dh: [
      [0.35, 0.00,  0],
      [0.00, 0.70,  0],
      [0.00, 0.50,  Math.PI],
      [0.20, 0.00,  0]
    ],
    limits: [[-135,135],[-135,135],[-180,180],[-180,180]]
  },
  '3link': {
    name: '3-Link Planar',
    dh: [
      [0.00, 0.80, 0],
      [0.00, 0.60, 0],
      [0.00, 0.40, 0]
    ],
    limits: [[-180,180],[-150,150],[-150,150]]
  }
};

class Robot {
  constructor(presetKey) {
    const p = PRESETS[presetKey];
    this.name = p.name;
    this.dh = p.dh;
    this.nJoints = p.dh.length;
    this.limits = p.limits.map(l => [l[0] * RAD, l[1] * RAD]);
    this.angles = new Float64Array(this.nJoints);
  }

  clampAngles(angles) {
    const out = new Float64Array(angles.length);
    for (let i = 0; i < this.nJoints; i++) {
      out[i] = Math.max(this.limits[i][0], Math.min(this.limits[i][1], angles[i]));
    }
    return out;
  }

  fk(angles) {
    const transforms = [mat4Identity()];
    for (let i = 0; i < this.nJoints; i++) {
      const [d, a, alpha] = this.dh[i];
      const T = dhTransform(angles[i], d, a, alpha);
      transforms.push(mat4Mul(transforms[i], T));
    }
    return transforms;
  }

  jacobian(angles) {
    const T = this.fk(angles);
    const pe = mat4Pos(T[this.nJoints]);
    const n = this.nJoints;
    const Jp = new Float64Array(3 * n); // position rows
    const Jr = new Float64Array(3 * n); // orientation rows
    for (let i = 0; i < n; i++) {
      const z = mat4AxisZ(T[i]);
      const p = mat4Pos(T[i]);
      const d = v3Sub(pe, p);
      const cp = v3Cross(z, d);
      Jp[0 * n + i] = cp[0]; Jp[1 * n + i] = cp[1]; Jp[2 * n + i] = cp[2];
      Jr[0 * n + i] = z[0];  Jr[1 * n + i] = z[1];  Jr[2 * n + i] = z[2];
    }
    return { Jp, Jr, n };
  }

  solveIK(target, startAngles, opts = {}) {
    const maxIter = opts.maxIter || 200;
    const tol = opts.tol || 0.001;
    const lambda = opts.lambda || 0.5;
    const n = this.nJoints;
    let angles = new Float64Array(startAngles || this.angles);

    for (let iter = 0; iter < maxIter; iter++) {
      const T = this.fk(angles);
      const pe = mat4Pos(T[n]);
      const err = v3Sub(target, pe);
      const dist = v3Len(err);
      if (dist < tol) return { angles: this.clampAngles(angles), converged: true, error: dist, iterations: iter };

      const { Jp } = this.jacobian(angles);
      // Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e
      // M = J J^T + lambda^2 I  (3x3)
      const M = new Float64Array(9);
      for (let r = 0; r < 3; r++)
        for (let c = 0; c < 3; c++) {
          let s = 0;
          for (let k = 0; k < n; k++) s += Jp[r * n + k] * Jp[c * n + k];
          M[r * 3 + c] = s + (r === c ? lambda * lambda : 0);
        }
      const v = solveLinear(M, [err[0], err[1], err[2]], 3);
      // dq = J^T v
      for (let i = 0; i < n; i++) {
        let dq = 0;
        for (let r = 0; r < 3; r++) dq += Jp[r * n + i] * v[r];
        angles[i] += dq;
      }
      angles = this.clampAngles(angles);
    }
    const T = this.fk(angles);
    const pe = mat4Pos(T[n]);
    return { angles, converged: false, error: v3Len(v3Sub(target, pe)), iterations: maxIter };
  }
}

// ================================================================
//  TEST SUITE
// ================================================================

function formatVector(v) {
  return `[${v[0].toFixed(4)}, ${v[1].toFixed(4)}, ${v[2].toFixed(4)}]`;
}

function runTests() {
  console.log('\n' + '='.repeat(70));
  console.log('ROBOT KINEMATICS VERIFICATION TEST SUITE');
  console.log('='.repeat(70) + '\n');

  let passCount = 0;
  let failCount = 0;

  // Test 1: Forward kinematics at zero angles
  console.log('TEST 1: Forward Kinematics at Zero Angles');
  console.log('-'.repeat(70));
  try {
    const robot = new Robot('6dof');
    const angles = new Float64Array(robot.nJoints);
    const transforms = robot.fk(angles);
    const ee = mat4Pos(transforms[robot.nJoints]);
    
    const valid = isFinite(ee[0]) && isFinite(ee[1]) && isFinite(ee[2]);
    const distance = v3Len(ee);
    
    console.log(`Robot: ${robot.name}`);
    console.log(`End-effector position: ${formatVector(ee)}`);
    console.log(`Distance from origin: ${distance.toFixed(4)} m`);
    console.log(`All values finite: ${valid ? 'YES' : 'NO'}`);
    console.log(`Distance reasonable (>0.5m): ${distance > 0.5 ? 'YES' : 'NO'}`);
    
    if (valid && distance > 0.5) {
      console.log('RESULT: PASS\n');
      passCount++;
    } else {
      console.log('RESULT: FAIL\n');
      failCount++;
    }
  } catch (e) {
    console.log(`ERROR: ${e.message}`);
    console.log('RESULT: FAIL\n');
    failCount++;
  }

  // Test 2: IK solver finds solution for reachable target
  console.log('TEST 2: IK Solver - Reachable Target');
  console.log('-'.repeat(70));
  try {
    const robot = new Robot('6dof');
    
    // Set up a simple configuration
    const startAngles = new Float64Array(robot.nJoints);
    startAngles[1] = -45 * RAD;
    startAngles[2] = 45 * RAD;
    
    const transforms = robot.fk(startAngles);
    const targetEE = mat4Pos(transforms[robot.nJoints]);
    
    console.log(`Start angles (deg): [${Array.from(startAngles).map(a => (a*DEG).toFixed(1)).join(', ')}]`);
    console.log(`Target position: ${formatVector(targetEE)}`);
    
    // Run IK solver
    const result = robot.solveIK(targetEE, startAngles, { maxIter: 200, tol: 0.001, lambda: 0.5 });
    
    console.log(`Converged: ${result.converged ? 'YES' : 'NO'}`);
    console.log(`Iterations: ${result.iterations}`);
    console.log(`Final error: ${result.error.toFixed(6)} m`);
    
    if (result.error < 0.01) {
      console.log('RESULT: PASS\n');
      passCount++;
    } else {
      console.log('RESULT: FAIL\n');
      failCount++;
    }
  } catch (e) {
    console.log(`ERROR: ${e.message}`);
    console.log('RESULT: FAIL\n');
    failCount++;
  }

  // Test 3: FK of IK solution matches target
  console.log('TEST 3: FK(IK(target)) = target Verification');
  console.log('-'.repeat(70));
  try {
    const robot = new Robot('6dof');
    
    // Create a random reachable target
    const randomAngles = new Float64Array(robot.nJoints);
    randomAngles[0] = (Math.random() - 0.5) * Math.PI;
    randomAngles[1] = -90 * RAD + Math.random() * 45 * RAD;
    randomAngles[2] = 20 * RAD + Math.random() * 40 * RAD;
    randomAngles[3] = (Math.random() - 0.5) * Math.PI;
    randomAngles[4] = (Math.random() - 0.5) * Math.PI * 0.5;
    randomAngles[5] = (Math.random() - 0.5) * Math.PI;
    
    // Clamp angles to limits
    const clampedAngles = robot.clampAngles(randomAngles);
    
    // Forward kinematics to get target
    const fkTransforms = robot.fk(clampedAngles);
    const target = mat4Pos(fkTransforms[robot.nJoints]);
    
    console.log(`Target position: ${formatVector(target)}`);
    console.log(`Starting angles (deg): [${Array.from(clampedAngles).map(a => (a*DEG).toFixed(1)).join(', ')}]`);
    
    // Solve IK from zero angles
    const ikResult = robot.solveIK(target, new Float64Array(robot.nJoints), { maxIter: 300, tol: 0.001, lambda: 0.4 });
    
    // Apply IK solution through FK
    const ikTransforms = robot.fk(ikResult.angles);
    const recoveredEE = mat4Pos(ikTransforms[robot.nJoints]);
    
    const positionError = v3Len(v3Sub(target, recoveredEE));
    
    console.log(`IK solution (deg): [${Array.from(ikResult.angles).map(a => (a*DEG).toFixed(1)).join(', ')}]`);
    console.log(`IK converged: ${ikResult.converged ? 'YES' : 'NO'}`);
    console.log(`IK iterations: ${ikResult.iterations}`);
    console.log(`Recovered EE position: ${formatVector(recoveredEE)}`);
    console.log(`Position error: ${positionError.toFixed(6)} m`);
    console.log(`Target match tolerance: ${(positionError < 0.01 ? 'PASS' : 'FAIL')} (threshold: 0.01m)`);
    
    if (positionError < 0.01) {
      console.log('RESULT: PASS\n');
      passCount++;
    } else {
      console.log('RESULT: FAIL\n');
      failCount++;
    }
  } catch (e) {
    console.log(`ERROR: ${e.message}`);
    console.log('RESULT: FAIL\n');
    failCount++;
  }

  // Additional test with different robot type
  console.log('TEST 4: 3-Link Planar Robot Forward Kinematics');
  console.log('-'.repeat(70));
  try {
    const robot = new Robot('3link');
    const angles = new Float64Array(robot.nJoints);
    angles[0] = 0;
    angles[1] = 0;
    angles[2] = 0;
    
    const transforms = robot.fk(angles);
    const ee = mat4Pos(transforms[robot.nJoints]);
    
    // For planar 3-link with all zeros, EE should be at x = 0.8 + 0.6 + 0.4 = 1.8
    const expectedX = 1.8;
    const error = Math.abs(ee[0] - expectedX);
    
    console.log(`Robot: ${robot.name}`);
    console.log(`EE position: ${formatVector(ee)}`);
    console.log(`Expected X: ${expectedX.toFixed(3)}, Actual: ${ee[0].toFixed(3)}, Error: ${error.toFixed(6)}`);
    
    if (error < 0.001) {
      console.log('RESULT: PASS\n');
      passCount++;
    } else {
      console.log('RESULT: FAIL\n');
      failCount++;
    }
  } catch (e) {
    console.log(`ERROR: ${e.message}`);
    console.log('RESULT: FAIL\n');
    failCount++;
  }

  // Summary
  console.log('='.repeat(70));
  console.log('TEST SUMMARY');
  console.log('='.repeat(70));
  console.log(`PASSED: ${passCount}`);
  console.log(`FAILED: ${failCount}`);
  console.log(`TOTAL:  ${passCount + failCount}`);
  console.log('='.repeat(70) + '\n');
  
  process.exit(failCount > 0 ? 1 : 0);
}

// Run tests
runTests();
