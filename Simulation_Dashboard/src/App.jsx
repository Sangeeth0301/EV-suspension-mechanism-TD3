import React, { useState, useEffect, useRef } from 'react';
import { 
  LineChart, Line, AreaChart, Area, XAxis, YAxis, CartesianGrid, 
  Tooltip, Legend, ResponsiveContainer, ReferenceLine
} from 'recharts';
import { Activity, Battery, Shield, ShieldAlert, Zap, Car, Play, Pause, RotateCcw, Repeat, Package } from 'lucide-react';
import MetricCard from './components/MetricCard';
import ChartPanel from './components/ChartPanel';
import CarVisualizer from './components/CarVisualizer';
import './index.css';

function App() {
  const [data, setData] = useState([]);
  const [chartData, setChartData] = useState([]);
  const [loading, setLoading] = useState(true);
  
  // Playback State
  const [isPlaying, setIsPlaying] = useState(false);
  const [isLooping, setIsLooping] = useState(false);
  const [currentTime, setCurrentTime] = useState(0.0);
  const [speedMultiplier, setSpeedMultiplier] = useState(1);
  const animationRef = useRef(null);

  useEffect(() => {
    // Add cache buster to ensure the browser loads the latest physics data
    fetch(`/sim_data.json?v=${Date.now()}`)
      .then(response => response.json())
      .then(jsonData => {
        const formattedData = jsonData.map(d => ({
          ...d,
          w_f_mm: d.w_f * 1000, 
          rho_dot_scaled: d.rho_dot_f * 0.1,
          time_ms: (d.time * 1000).toFixed(0)
        }));
        setData(formattedData);
        // Downsample for Recharts to prevent SVG overload (10,000 nodes -> 500 nodes)
        setChartData(formattedData.filter((_, index) => index % 20 === 0));
        setLoading(false);
      })
      .catch(error => {
        console.error("Error loading simulation data:", error);
        setLoading(false);
      });
  }, []);

  // High-performance animation loop
  useEffect(() => {
    let lastTimestamp = null;

    const animate = (now) => {
      if (!isPlaying) return;
      if (lastTimestamp === null) lastTimestamp = now;
      const deltaSeconds = ((now - lastTimestamp) / 1000) * speedMultiplier;
      lastTimestamp = now;

      setCurrentTime((prevTime) => {
        const nextTime = prevTime + deltaSeconds;
        if (nextTime >= 10.0) {
          if (isLooping) {
            return 0.0;
          } else {
            setIsPlaying(false);
            return 10.0;
          }
        }
        return nextTime;
      });

      animationRef.current = requestAnimationFrame(animate);
    };

    if (isPlaying) {
      animationRef.current = requestAnimationFrame(animate);
    }

    return () => cancelAnimationFrame(animationRef.current);
  }, [isPlaying, speedMultiplier, isLooping]);

  if (loading) {
    return (
      <div className="loading-container">
        <div className="spinner"></div>
        <p>Loading Simulation Data...</p>
      </div>
    );
  }

  // Fast O(1) lookup instead of O(N) Array.find running at 60 FPS
  const frameIndex = Math.min(Math.floor(currentTime * 1000), Math.max(0, data.length - 1));
  const currentFrame = data[frameIndex] || data[0];
  const isAttack = currentFrame?.cyber_attack || false;
  const isEco = currentFrame?.instant_power_w > 100;

  return (
    <div className="dashboard-container">
      <header className="dashboard-header">
        <div>
          <h1 className="header-title">Cyber-Resilient Active Suspension</h1>
          <p className="header-subtitle">LPV-Adaptive Control & TD3 Energy Harvester for IWM-EVs</p>
        </div>
      </header>

      {/* 1. CONTROL DECK & TELEMETRY */}
      <section className="control-deck" style={{
        background: 'var(--bg-panel)',
        borderRadius: '16px',
        padding: '1.5rem',
        border: '1px solid var(--glass-border)',
        display: 'flex',
        flexDirection: 'column',
        gap: '1rem',
        marginBottom: '1rem'
      }}>
        
        {/* Top row: Buttons and Scrubber */}
        <div style={{ display: 'flex', alignItems: 'center', gap: '2rem' }}>
          <div style={{ display: 'flex', gap: '0.5rem' }}>
            <button 
              onClick={() => setIsPlaying(!isPlaying)}
              style={{ background: 'var(--accent-blue)', color: 'white', border: 'none', padding: '0.75rem 1.5rem', borderRadius: '8px', display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontWeight: 600 }}
            >
              {isPlaying ? <Pause size={18} /> : <Play size={18} />}
              {isPlaying ? 'PAUSE' : 'PLAY'}
            </button>
            <button 
              onClick={() => { setIsPlaying(false); setCurrentTime(0); }}
              style={{ background: 'transparent', color: 'var(--text-secondary)', border: '1px solid var(--text-secondary)', padding: '0.75rem 1.5rem', borderRadius: '8px', display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontWeight: 600 }}
            >
              <RotateCcw size={18} /> RESET
            </button>
            <button 
              onClick={() => setIsLooping(!isLooping)}
              style={{ background: isLooping ? 'var(--accent-blue)' : 'transparent', color: isLooping ? 'white' : 'var(--text-secondary)', border: `1px solid ${isLooping ? 'var(--accent-blue)' : 'var(--text-secondary)'}`, padding: '0.75rem 1.5rem', borderRadius: '8px', display: 'flex', alignItems: 'center', gap: '0.5rem', cursor: 'pointer', fontWeight: 600 }}
            >
              <Repeat size={18} /> LOOP
            </button>
          </div>
          
          <div style={{ flex: 1, display: 'flex', alignItems: 'center', gap: '1rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontFamily: 'monospace' }}>0.0s</span>
            <input 
              type="range" 
              min="0" 
              max="10" 
              step="0.01" 
              value={currentTime}
              onChange={(e) => {
                setIsPlaying(false);
                setCurrentTime(parseFloat(e.target.value));
              }}
              style={{ flex: 1, accentColor: 'var(--accent-blue)' }}
            />
            <span style={{ color: 'var(--text-secondary)', fontFamily: 'monospace' }}>10.0s</span>
          </div>

          <div style={{ display: 'flex', alignItems: 'center', gap: '0.5rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.875rem' }}>SPEED:</span>
            <select 
              value={speedMultiplier} 
              onChange={(e) => setSpeedMultiplier(parseFloat(e.target.value))}
              style={{ background: 'transparent', color: 'white', border: '1px solid var(--glass-border)', padding: '0.5rem', borderRadius: '4px' }}
            >
              <option value="0.5">0.5x</option>
              <option value="1">1x</option>
              <option value="2">2x</option>
              <option value="5">5x</option>
            </select>
          </div>
        </div>

        {/* Bottom row: Telemetry Pills */}
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: '1rem', padding: '1rem', background: 'rgba(0,0,0,0.3)', borderRadius: '8px' }}>
          
          {/* Mode Badge */}
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.75rem', letterSpacing: '1px' }}>ACTIVE MODE</span>
            <div style={{ 
              padding: '0.5rem 1rem', 
              borderRadius: '999px', 
              display: 'flex', alignItems: 'center', gap: '0.5rem',
              background: isEco ? 'rgba(16, 185, 129, 0.1)' : 'rgba(59, 130, 246, 0.1)',
              border: `1px solid ${isEco ? '#10b981' : '#3b82f6'}`,
              color: isEco ? '#10b981' : '#3b82f6',
              fontWeight: 600
            }}>
              {isEco ? <Zap size={16} /> : <Activity size={16} />}
              {isEco ? 'ECO REGEN' : 'COMFORT'}
            </div>
          </div>

          {/* Network Badge */}
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.75rem', letterSpacing: '1px' }}>NETWORK STATUS</span>
            <div style={{ 
              padding: '0.5rem 1rem', 
              borderRadius: '999px', 
              display: 'flex', alignItems: 'center', gap: '0.5rem',
              background: isAttack ? 'rgba(239, 68, 68, 0.2)' : 'rgba(59, 130, 246, 0.1)',
              border: `1px solid ${isAttack ? '#ef4444' : '#3b82f6'}`,
              color: isAttack ? '#ef4444' : '#3b82f6',
              fontWeight: 600,
              animation: isAttack ? 'pulse 1s infinite' : 'none'
            }}>
              {isAttack ? <ShieldAlert size={16} /> : <Shield size={16} />}
              {isAttack ? 'ETHERNET FAILOVER (ATTACK)' : 'CAN BUS (METS)'}
            </div>
          </div>

          {/* Actuator Force */}
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.75rem', letterSpacing: '1px' }}>FORCE</span>
            <div style={{ 
              padding: '0.5rem 1rem', 
              borderRadius: '999px', 
              display: 'flex', alignItems: 'center', gap: '0.5rem',
              background: 'rgba(139, 92, 246, 0.1)',
              border: '1px solid #8b5cf6',
              color: '#8b5cf6',
              fontWeight: 600,
              fontFamily: 'monospace'
            }}>
              <Car size={16} />
              {currentFrame.u_f.toFixed(0)} N
            </div>
          </div>

          {/* Time Scrubber Exact */}
          <div style={{ color: 'white', fontFamily: 'monospace', fontSize: '1.25rem' }}>
            t = {currentTime.toFixed(2)}s
          </div>

        </div>
      </section>

      {/* 2. LIVE KPI CARDS */}
      <section className="metrics-grid">
        <MetricCard 
          title="Comfort (RMS Accel)" 
          value={Math.abs(currentFrame.z_c).toFixed(3)} 
          unit="m" 
          icon={Activity} 
        />
        <MetricCard 
          title="Base Comfort (No AI)" 
          value={Math.abs(currentFrame.base_z_c).toFixed(3)} 
          unit="m" 
          icon={Car} 
        />
        <MetricCard 
          title="48V Aux Battery" 
          value={currentFrame.net_battery_kwh ? currentFrame.net_battery_kwh.toFixed(4) : "1.5000"} 
          unit="kWh" 
          icon={Battery} 
        />
        <MetricCard 
          title="Remaining Range" 
          value={currentFrame.remaining_range_km ? currentFrame.remaining_range_km.toFixed(2) : "333.33"} 
          unit="km" 
          icon={Car} 
        />
        <MetricCard 
          title="Range Saved by Suspension" 
          value={currentFrame.harvested_range_m ? (currentFrame.harvested_range_m / 1000).toFixed(4) : "0.0000"} 
          unit="km" 
          icon={Zap} 
          highlight={currentFrame.harvested_range_m > 0}
        />
        <MetricCard 
          title="Added PE Mass" 
          value="9.00" 
          unit="kg" 
          icon={Package} 
        />
        {/* Dual Metric: Instant Power */}
        <div className={`metric-card ${currentFrame.instant_power_w > 500 ? 'highlight' : ''}`}>
          <div className="metric-header">
            <h3 className="metric-title">Instant Power</h3>
            <Zap className="metric-icon" size={20} color={currentFrame.instant_power_w > 500 ? '#10b981' : 'var(--text-secondary)'} />
          </div>
          <div className="metric-value-container">
            <span className="metric-value" style={{ color: currentFrame.instant_power_w > 500 ? '#10b981' : 'var(--text-primary)'}}>
              {currentFrame.instant_power_w > 0 ? `+${currentFrame.instant_power_w.toFixed(0)}` : '0'}
            </span>
            <span className="metric-unit">W PEAK</span>
          </div>
        </div>
      </section>

      {/* 3. CHARTS GRID */}
      <section className="charts-grid">
        {/* Panel 1: Road Profile */}
        <ChartPanel colSpan={2} title="1. Road Profile (Mixed Pothole Track)" subtitle="ISO Class A + Random Deterministic Potholes">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <defs>
                <linearGradient id="colorRoad" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#94a3b8" stopOpacity={0.3}/>
                  <stop offset="95%" stopColor="#94a3b8" stopOpacity={0}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" tickFormatter={(val) => `${val.toFixed(1)}s`} />
              <YAxis stroke="#94a3b8" unit="mm" />
              <Tooltip />
              <Area type="monotone" dataKey="w_f_mm" name="Road Height" stroke="#94a3b8" fillOpacity={1} fill="url(#colorRoad)" isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </AreaChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 2: UKF Estimator */}
        <ChartPanel title="2. UKF Road Estimation" subtitle="Severity (ρ) and Rate of Change (ρ̇) for Feedforward">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" />
              <Tooltip />
              <Legend />
              <Line type="stepAfter" dataKey="rho_f" name="ρ (Severity)" stroke="#ef4444" strokeWidth={2} dot={false} isAnimationActive={false} />
              <Line type="monotone" dataKey="rho_dot_scaled" name="0.1 × ρ̇ (Rate)" stroke="#f59e0b" strokeWidth={2} strokeDasharray="5 5" dot={false} isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </LineChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 3: LPV Actuator Force */}
        <ChartPanel title="3. LPV Actuator Force" subtitle="Active force applied by the motor to counter the bumps">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <defs>
                <linearGradient id="colorForce" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#8b5cf6" stopOpacity={0.3}/>
                  <stop offset="95%" stopColor="#8b5cf6" stopOpacity={0}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" unit="N" />
              <Tooltip />
              <Area type="monotone" dataKey="u_f" name="Actuator Force (u)" stroke="#8b5cf6" strokeWidth={2} fillOpacity={1} fill="url(#colorForce)" isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </AreaChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 4: 2D Car Visualizer */}
        <ChartPanel colSpan={2} title="4. Digital Twin: Real-Time Suspension Kinematics" subtitle="Visualizing the AI absorbing road severity while holding the cabin steady">
          <div style={{ width: '100%', height: '100%', minHeight: '300px' }}>
            <CarVisualizer currentFrame={currentFrame} data={data} currentTime={currentTime} />
          </div>
        </ChartPanel>
      </section>

      {/* 4. ANALYSIS SECTION */}
      <section className="charts-grid" style={{ marginTop: '1.5rem' }}>
        <ChartPanel title="5. Mechanical Damping vs Active Control" subtitle="Bode Plot Analysis of LQR Virtual Damping">
          <div style={{ width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
            <img 
              src="/bode_plot.png" 
              alt="Bode Plot" 
              style={{ maxWidth: '100%', maxHeight: '400px', objectFit: 'contain', borderRadius: '8px' }} 
            />
          </div>
        </ChartPanel>

        <ChartPanel title="6. Regenerative Power Stabilization" subtitle="LC Filter + Shift Transformer (9.00 kg added mass)">
          <div style={{ width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
            <img 
              src="/power_stability_plot.png" 
              alt="Power Stability Plot" 
              style={{ maxWidth: '100%', maxHeight: '400px', objectFit: 'contain', borderRadius: '8px' }} 
            />
          </div>
        </ChartPanel>

        <ChartPanel colSpan={2} title="7. Control System Stability" subtitle="Pole-Zero Map of the LQR Closed-Loop System">
          <div style={{ width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
            <img 
              src="/pz_map.png" 
              alt="Pole Zero Map" 
              style={{ maxWidth: '100%', maxHeight: '500px', objectFit: 'contain', borderRadius: '8px', background: 'white' }} 
            />
          </div>
        </ChartPanel>
      </section>

      <style>{`
        @keyframes pulse {
          0% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0.7); }
          70% { box-shadow: 0 0 0 10px rgba(239, 68, 68, 0); }
          100% { box-shadow: 0 0 0 0 rgba(239, 68, 68, 0); }
        }
      `}</style>
    </div>
  );
}

export default App;
