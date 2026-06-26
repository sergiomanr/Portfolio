// Generated Physics Equations of Motion (Lagrangian Mechanics)
// Derived symbolically using SymPy

function getEOM_2pend(q, qd, params, f_input) {
  // Unpack states (q: [x, a1, a2, ...], qd: [xd, ad1, ad2, ...])
  const q_0 = q[0];
  const qd_0 = qd[0];
  const q_1 = q[1];
  const qd_1 = qd[1];
  const q_2 = q[2];
  const qd_2 = qd[2];

  // Unpack params
  const M_c = params.M_c;
  const m1 = params.m1;
  const I1 = params.I1;
  const m2 = params.m2;
  const I2 = params.I2;
  const L = params.L;
  const g = params.g;
  const d_c = params.d_c;
  const d_p = params.d_p;
  const f = f_input;

  const A = [];
  A[0] = [];
  A[0][0] = 1.0*M_c + 1.0*m1 + 1.0*m2;
  A[0][1] = L*(0.5*m1 + 1.0*m2)*Math.cos(q_1);
  A[0][2] = 0.5*L*m2*Math.cos(q_2);
  A[1] = [];
  A[1][0] = L*(0.5*m1 + 1.0*m2)*Math.cos(q_1);
  A[1][1] = 1.0*I1 + 0.25*L**2*m1 + 1.0*L**2*m2;
  A[1][2] = 0.5*L**2*m2*Math.cos(q_1 - q_2);
  A[2] = [];
  A[2][0] = 0.5*L*m2*Math.cos(q_2);
  A[2][1] = 0.5*L**2*m2*Math.cos(q_1 - q_2);
  A[2][2] = 1.0*I2 + 0.25*L**2*m2;

  const B = [];
  B[0] = 0.5*L*m2*qd_2**2*Math.sin(q_2) + L*qd_1**2*(0.5*m1 + 1.0*m2)*Math.sin(q_1) - d_c*qd_0 + f;
  B[1] = -0.5*L**2*m2*qd_2**2*Math.sin(q_1 - q_2) + 0.5*L*g*m1*Math.sin(q_1) + 1.0*L*g*m2*Math.sin(q_1) - 2.0*d_p*qd_1 + 1.0*d_p*qd_2;
  B[2] = 0.5*L**2*m2*qd_1**2*Math.sin(q_1 - q_2) + L*g*m2*Math.sin(q_2)/2 + d_p*(qd_1 - qd_2);

  return { A, B };
}

function getEOM_3pend(q, qd, params, f_input) {
  // Unpack states (q: [x, a1, a2, ...], qd: [xd, ad1, ad2, ...])
  const q_0 = q[0];
  const qd_0 = qd[0];
  const q_1 = q[1];
  const qd_1 = qd[1];
  const q_2 = q[2];
  const qd_2 = qd[2];
  const q_3 = q[3];
  const qd_3 = qd[3];

  // Unpack params
  const M_c = params.M_c;
  const m1 = params.m1;
  const I1 = params.I1;
  const m2 = params.m2;
  const I2 = params.I2;
  const m3 = params.m3;
  const I3 = params.I3;
  const L = params.L;
  const g = params.g;
  const d_c = params.d_c;
  const d_p = params.d_p;
  const f = f_input;

  const A = [];
  A[0] = [];
  A[0][0] = 1.0*M_c + 1.0*m1 + 1.0*m2 + 1.0*m3;
  A[0][1] = L*(0.5*m1 + 1.0*m2 + 1.0*m3)*Math.cos(q_1);
  A[0][2] = L*(0.5*m2 + 1.0*m3)*Math.cos(q_2);
  A[0][3] = 0.5*L*m3*Math.cos(q_3);
  A[1] = [];
  A[1][0] = L*(0.5*m1 + 1.0*m2 + 1.0*m3)*Math.cos(q_1);
  A[1][1] = 1.0*I1 + 0.25*L**2*m1 + 1.0*L**2*m2 + 1.0*L**2*m3;
  A[1][2] = L**2*(0.5*m2 + 1.0*m3)*Math.cos(q_1 - q_2);
  A[1][3] = 0.5*L**2*m3*Math.cos(q_1 - q_3);
  A[2] = [];
  A[2][0] = L*(0.5*m2 + 1.0*m3)*Math.cos(q_2);
  A[2][1] = L**2*(0.5*m2 + 1.0*m3)*Math.cos(q_1 - q_2);
  A[2][2] = 1.0*I2 + 0.25*L**2*m2 + 1.0*L**2*m3;
  A[2][3] = 0.5*L**2*m3*Math.cos(q_2 - q_3);
  A[3] = [];
  A[3][0] = 0.5*L*m3*Math.cos(q_3);
  A[3][1] = 0.5*L**2*m3*Math.cos(q_1 - q_3);
  A[3][2] = 0.5*L**2*m3*Math.cos(q_2 - q_3);
  A[3][3] = 1.0*I3 + 0.25*L**2*m3;

  const B = [];
  B[0] = 0.5*L*m3*qd_3**2*Math.sin(q_3) + L*qd_1**2*(0.5*m1 + 1.0*m2 + 1.0*m3)*Math.sin(q_1) + L*qd_2**2*(0.5*m2 + 1.0*m3)*Math.sin(q_2) - d_c*qd_0 + f;
  B[1] = -0.5*L**2*m2*qd_2**2*Math.sin(q_1 - q_2) - 1.0*L**2*m3*qd_2**2*Math.sin(q_1 - q_2) - 0.5*L**2*m3*qd_3**2*Math.sin(q_1 - q_3) + 0.5*L*g*m1*Math.sin(q_1) + 1.0*L*g*m2*Math.sin(q_1) + 1.0*L*g*m3*Math.sin(q_1) - 2.0*d_p*qd_1 + 1.0*d_p*qd_2;
  B[2] = 0.5*L**2*m2*qd_1**2*Math.sin(q_1 - q_2) + 1.0*L**2*m3*qd_1**2*Math.sin(q_1 - q_2) - 0.5*L**2*m3*qd_3**2*Math.sin(q_2 - q_3) + 0.5*L*g*m2*Math.sin(q_2) + 1.0*L*g*m3*Math.sin(q_2) + 1.0*d_p*qd_1 - 2.0*d_p*qd_2 + 1.0*d_p*qd_3;
  B[3] = 0.5*L**2*m3*qd_1**2*Math.sin(q_1 - q_3) + 0.5*L**2*m3*qd_2**2*Math.sin(q_2 - q_3) + L*g*m3*Math.sin(q_3)/2 + d_p*(qd_2 - qd_3);

  return { A, B };
}

function getEOM_4pend(q, qd, params, f_input) {
  // Unpack states (q: [x, a1, a2, ...], qd: [xd, ad1, ad2, ...])
  const q_0 = q[0];
  const qd_0 = qd[0];
  const q_1 = q[1];
  const qd_1 = qd[1];
  const q_2 = q[2];
  const qd_2 = qd[2];
  const q_3 = q[3];
  const qd_3 = qd[3];
  const q_4 = q[4];
  const qd_4 = qd[4];

  // Unpack params
  const M_c = params.M_c;
  const m1 = params.m1;
  const I1 = params.I1;
  const m2 = params.m2;
  const I2 = params.I2;
  const m3 = params.m3;
  const I3 = params.I3;
  const m4 = params.m4;
  const I4 = params.I4;
  const L = params.L;
  const g = params.g;
  const d_c = params.d_c;
  const d_p = params.d_p;
  const f = f_input;

  const A = [];
  A[0] = [];
  A[0][0] = 1.0*M_c + 1.0*m1 + 1.0*m2 + 1.0*m3 + 1.0*m4;
  A[0][1] = L*(0.5*m1 + 1.0*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_1);
  A[0][2] = L*(0.5*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_2);
  A[0][3] = L*(0.5*m3 + 1.0*m4)*Math.cos(q_3);
  A[0][4] = 0.5*L*m4*Math.cos(q_4);
  A[1] = [];
  A[1][0] = L*(0.5*m1 + 1.0*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_1);
  A[1][1] = 1.0*I1 + 0.25*L**2*m1 + 1.0*L**2*m2 + 1.0*L**2*m3 + 1.0*L**2*m4;
  A[1][2] = L**2*(0.5*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_1 - q_2);
  A[1][3] = L**2*(0.5*m3 + 1.0*m4)*Math.cos(q_1 - q_3);
  A[1][4] = 0.5*L**2*m4*Math.cos(q_1 - q_4);
  A[2] = [];
  A[2][0] = L*(0.5*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_2);
  A[2][1] = L**2*(0.5*m2 + 1.0*m3 + 1.0*m4)*Math.cos(q_1 - q_2);
  A[2][2] = 1.0*I2 + 0.25*L**2*m2 + 1.0*L**2*m3 + 1.0*L**2*m4;
  A[2][3] = L**2*(0.5*m3 + 1.0*m4)*Math.cos(q_2 - q_3);
  A[2][4] = 0.5*L**2*m4*Math.cos(q_2 - q_4);
  A[3] = [];
  A[3][0] = L*(0.5*m3 + 1.0*m4)*Math.cos(q_3);
  A[3][1] = L**2*(0.5*m3 + 1.0*m4)*Math.cos(q_1 - q_3);
  A[3][2] = L**2*(0.5*m3 + 1.0*m4)*Math.cos(q_2 - q_3);
  A[3][3] = 1.0*I3 + 0.25*L**2*m3 + 1.0*L**2*m4;
  A[3][4] = 0.5*L**2*m4*Math.cos(q_3 - q_4);
  A[4] = [];
  A[4][0] = 0.5*L*m4*Math.cos(q_4);
  A[4][1] = 0.5*L**2*m4*Math.cos(q_1 - q_4);
  A[4][2] = 0.5*L**2*m4*Math.cos(q_2 - q_4);
  A[4][3] = 0.5*L**2*m4*Math.cos(q_3 - q_4);
  A[4][4] = 1.0*I4 + 0.25*L**2*m4;

  const B = [];
  B[0] = 0.5*L*m4*qd_4**2*Math.sin(q_4) + L*qd_1**2*(0.5*m1 + 1.0*m2 + 1.0*m3 + 1.0*m4)*Math.sin(q_1) + L*qd_2**2*(0.5*m2 + 1.0*m3 + 1.0*m4)*Math.sin(q_2) + L*qd_3**2*(0.5*m3 + 1.0*m4)*Math.sin(q_3) - d_c*qd_0 + f;
  B[1] = -0.5*L**2*m2*qd_2**2*Math.sin(q_1 - q_2) - 1.0*L**2*m3*qd_2**2*Math.sin(q_1 - q_2) - 0.5*L**2*m3*qd_3**2*Math.sin(q_1 - q_3) - 1.0*L**2*m4*qd_2**2*Math.sin(q_1 - q_2) - 1.0*L**2*m4*qd_3**2*Math.sin(q_1 - q_3) - 0.5*L**2*m4*qd_4**2*Math.sin(q_1 - q_4) + 0.5*L*g*m1*Math.sin(q_1) + 1.0*L*g*m2*Math.sin(q_1) + 1.0*L*g*m3*Math.sin(q_1) + 1.0*L*g*m4*Math.sin(q_1) - 2.0*d_p*qd_1 + 1.0*d_p*qd_2;
  B[2] = 0.5*L**2*m2*qd_1**2*Math.sin(q_1 - q_2) + 1.0*L**2*m3*qd_1**2*Math.sin(q_1 - q_2) - 0.5*L**2*m3*qd_3**2*Math.sin(q_2 - q_3) + 1.0*L**2*m4*qd_1**2*Math.sin(q_1 - q_2) - 1.0*L**2*m4*qd_3**2*Math.sin(q_2 - q_3) - 0.5*L**2*m4*qd_4**2*Math.sin(q_2 - q_4) + 0.5*L*g*m2*Math.sin(q_2) + 1.0*L*g*m3*Math.sin(q_2) + 1.0*L*g*m4*Math.sin(q_2) + 1.0*d_p*qd_1 - 2.0*d_p*qd_2 + 1.0*d_p*qd_3;
  B[3] = 0.5*L**2*m3*qd_1**2*Math.sin(q_1 - q_3) + 0.5*L**2*m3*qd_2**2*Math.sin(q_2 - q_3) + 1.0*L**2*m4*qd_1**2*Math.sin(q_1 - q_3) + 1.0*L**2*m4*qd_2**2*Math.sin(q_2 - q_3) - 0.5*L**2*m4*qd_4**2*Math.sin(q_3 - q_4) + 0.5*L*g*m3*Math.sin(q_3) + 1.0*L*g*m4*Math.sin(q_3) + 1.0*d_p*qd_2 - 2.0*d_p*qd_3 + 1.0*d_p*qd_4;
  B[4] = 0.5*L**2*m4*qd_1**2*Math.sin(q_1 - q_4) + 0.5*L**2*m4*qd_2**2*Math.sin(q_2 - q_4) + 0.5*L**2*m4*qd_3**2*Math.sin(q_3 - q_4) + L*g*m4*Math.sin(q_4)/2 + d_p*(qd_3 - qd_4);

  return { A, B };
}
