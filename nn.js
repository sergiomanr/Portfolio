class PolicyNet {
  constructor(name, modelData) {
    this.name = name;
    this.metadata = modelData.metadata;
    this.weights = modelData.weights;
    
    // Determine dimensions
    this.obsDim = this.metadata["encoder.weight"].shape[1];
    this.hiddenDim = this.metadata["encoder.weight"].shape[0];
    this.actDim = this.metadata["actor.weight"].shape[0];
    
    // Allocate recurrent state
    this.hiddenState = new Float32Array(this.hiddenDim);
  }
  
  reset() {
    this.hiddenState.fill(0);
  }
  
  forward(obs, useRecurrence) {
    const H = this.hiddenDim;
    const x = new Float32Array(H);
    
    // 1. Encoder forward pass
    const encoderWeight = this.weights["encoder.weight"];
    const encoderBias = this.weights["encoder.bias"];
    
    if (this.name === "4pend") {
      // 4pend uses: tanh(LayerNorm(W * obs + b))
      const x_lin = new Float32Array(H);
      for (let r = 0; r < H; r++) {
        let sum = encoderBias[r];
        const offset = r * this.obsDim;
        for (let c = 0; c < this.obsDim; c++) {
          sum += encoderWeight[offset + c] * obs[c];
        }
        x_lin[r] = sum;
      }
      
      // LayerNorm calculation
      let sum = 0;
      for (let i = 0; i < H; i++) {
        sum += x_lin[i];
      }
      const mean = sum / H;
      
      let varianceSum = 0;
      for (let i = 0; i < H; i++) {
        const diff = x_lin[i] - mean;
        varianceSum += diff * diff;
      }
      const variance = varianceSum / H;
      const std = Math.sqrt(variance + 1e-5);
      
      const lnWeight = this.weights["enc_norm.weight"];
      const lnBias = this.weights["enc_norm.bias"];
      
      for (let i = 0; i < H; i++) {
        const normalized = (x_lin[i] - mean) / std;
        x[i] = Math.tanh(normalized * lnWeight[i] + lnBias[i]);
      }
    } else {
      // 2pend and 3pend use: tanh(W * obs + b)
      for (let r = 0; r < H; r++) {
        let sum = encoderBias[r];
        const offset = r * this.obsDim;
        for (let c = 0; c < this.obsDim; c++) {
          sum += encoderWeight[offset + c] * obs[c];
        }
        x[r] = Math.tanh(sum);
      }
    }
    
    // 2. minGRU Cell recurrence
    const h = useRecurrence ? this.hiddenState : new Float32Array(H);
    const hNext = new Float32Array(H);
    let gates = null; // to return gate activations for visual telemetry
    
    if (this.name === "4pend") {
      // 4pend minGRU Cell:
      // z = sigmoid(W_z * x)
      // h_tilde = W_h * x + b_h
      // h_next = (1 - z) * h + z * h_tilde
      const W_z = this.weights["min_gru.cell.W_z.weight"];
      const W_h = this.weights["min_gru.cell.W_h.weight"];
      const b_h = this.weights["min_gru.cell.W_h.bias"];
      
      gates = new Float32Array(H);
      
      for (let r = 0; r < H; r++) {
        let sum_z = 0;
        let sum_h = b_h[r];
        const offset = r * H;
        for (let c = 0; c < H; c++) {
          sum_z += W_z[offset + c] * x[c];
          sum_h += W_h[offset + c] * x[c];
        }
        
        const z = 1.0 / (1.0 + Math.exp(-sum_z));
        gates[r] = z;
        hNext[r] = (1.0 - z) * h[r] + z * sum_h;
      }
    } else {
      // 2pend and 3pend minGRU Cell (standard GRU cell formulas):
      // z = sigmoid(W_z * x)
      // r = sigmoid(W_r * x)
      // h_tilde = tanh(W_h * x + b_h + U_h * (r * h))
      // h_next = (1 - z) * h + z * h_tilde
      const W_z = this.weights["min_gru.cell.W_z.weight"];
      const W_r = this.weights["min_gru.cell.W_r.weight"];
      const W_h = this.weights["min_gru.cell.W_h.weight"];
      const b_h = this.weights["min_gru.cell.W_h.bias"];
      const U_h = this.weights["min_gru.cell.U_h.weight"];
      
      const z_gate = new Float32Array(H);
      const r_gate = new Float32Array(H);
      
      for (let r = 0; r < H; r++) {
        let sum_z = 0;
        let sum_r = 0;
        const offset = r * H;
        for (let c = 0; c < H; c++) {
          sum_z += W_z[offset + c] * x[c];
          sum_r += W_r[offset + c] * x[c];
        }
        z_gate[r] = 1.0 / (1.0 + Math.exp(-sum_z));
        r_gate[r] = 1.0 / (1.0 + Math.exp(-sum_r));
      }
      
      gates = z_gate; // we will display update gate activations
      
      const r_h = new Float32Array(H);
      for (let i = 0; i < H; i++) {
        r_h[i] = r_gate[i] * h[i];
      }
      
      const Uh_rh = new Float32Array(H);
      for (let r = 0; r < H; r++) {
        let sum = 0;
        const offset = r * H;
        for (let c = 0; c < H; c++) {
          sum += U_h[offset + c] * r_h[c];
        }
        Uh_rh[r] = sum;
      }
      
      for (let r = 0; r < H; r++) {
        let sum_wh = b_h[r];
        const offset = r * H;
        for (let c = 0; c < H; c++) {
          sum_wh += W_h[offset + c] * x[c];
        }
        const h_tilde = Math.tanh(sum_wh + Uh_rh[r]);
        hNext[r] = (1.0 - z_gate[r]) * h[r] + z_gate[r] * h_tilde;
      }
    }
    
    if (useRecurrence) {
      this.hiddenState.set(hNext);
    }
    
    // 3. Actor linear layer
    const actorWeight = this.weights["actor.weight"];
    const actorBias = this.weights["actor.bias"];
    let mean = actorBias[0];
    for (let c = 0; c < H; c++) {
      mean += actorWeight[c] * hNext[c];
    }
    
    const actionVal = Math.tanh(mean);
    
    return {
      action: actionVal,
      hidden: hNext,
      gates: gates
    };
  }
}
