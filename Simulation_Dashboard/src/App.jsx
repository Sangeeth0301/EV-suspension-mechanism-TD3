import React, { useState, useEffect, useRef } from 'react';
import { 
  LineChart, Line, AreaChart, Area, XAxis, YAxis, CartesianGrid, 
  Tooltip, Legend, ResponsiveContainer, ReferenceLine, ComposedChart, Bar
} from 'recharts';
import { Activity, Battery, Shield, ShieldAlert, Zap, Car, Play, Pause, RotateCcw, Repeat, Package, Gauge, TrendingUp, BatteryCharging } from 'lucide-react';
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
    fetch(`/sim_data.json?v=${Date.now()}`)
      .then(response => response.json())
      .then(jsonData => {
        const formattedData = jsonData.map(d => ({
          ...d,
          w_f_mm: d.w_f * 1000, 
          w_r_mm: (d.w_r || 0) * 1000,
          rho_dot_scaled: d.rho_dot_f * 0.1,
          time_ms: (d.time * 1000).toFixed(0),
          base_z_c_mm: d.base_z_c * 1000,
          z_c_mm: d.z_c * 1000,
          // Power fields
          power_raw_w: d.power_raw_w || 0,
          power_conditioned_w: d.power_conditioned_w || 0,
          power_consumed_w: d.power_consumed_w || 0,
          power_consumed_neg: -(d.power_consumed_w || 0),
          // Pitch
          theta_mrad: d.theta_mrad || 0,
          base_theta_mrad: d.base_theta_mrad || 0,
          // Energy balance
          cum_harvested_kj: d.cum_harvested_kj || 0,
          cum_consumed_kj_neg: -(d.cum_consumed_kj || 0),
          net_energy_kj: d.net_energy_kj || 0,
          // Mode
          mode_f: d.mode_f || 0,
          isEco: (d.mode_f || 0) > 0.5,
          // Lyapunov
          lyapunov_v: d.lyapunov_v || 0,
        }));
        setData(formattedData);
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

  // Fast O(1) lookup
  const frameIndex = Math.min(Math.floor(currentTime * 100), Math.max(0, data.length - 1));
  const currentFrame = data[frameIndex] || data[0];
  const isAttack = currentFrame?.cyber_attack || false;
  const isEco = currentFrame?.isEco || false;

  return (
    <div className="dashboard-container">
      <header className="dashboard-header">
        <div>
          <h1 className="header-title">Cyber-Resilient Active Suspension</h1>
          <p className="header-subtitle">LPV-Adaptive Control · TD3 Energy Harvester · Power Electronics · Lyapunov Stability</p>
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
        <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginTop: '1rem', padding: '1rem', background: 'rgba(0,0,0,0.3)', borderRadius: '8px', flexWrap: 'wrap', gap: '0.75rem' }}>
          
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
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.75rem', letterSpacing: '1px' }}>NETWORK</span>
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
              {isAttack ? 'ATTACK DETECTED' : 'CAN BUS (METS)'}
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
              {(currentFrame.u_f || 0).toFixed(0)} N
            </div>
          </div>

          {/* Regen Power */}
          <div style={{ display: 'flex', alignItems: 'center', gap: '0.75rem' }}>
            <span style={{ color: 'var(--text-secondary)', fontSize: '0.75rem', letterSpacing: '1px' }}>REGEN</span>
            <div style={{ 
              padding: '0.5rem 1rem', 
              borderRadius: '999px', 
              display: 'flex', alignItems: 'center', gap: '0.5rem',
              background: currentFrame.power_conditioned_w > 10 ? 'rgba(16, 185, 129, 0.15)' : 'rgba(100,100,100,0.1)',
              border: `1px solid ${currentFrame.power_conditioned_w > 10 ? '#10b981' : '#64748b'}`,
              color: currentFrame.power_conditioned_w > 10 ? '#10b981' : '#64748b',
              fontWeight: 600,
              fontFamily: 'monospace'
            }}>
              <BatteryCharging size={16} />
              +{(currentFrame.power_conditioned_w || 0).toFixed(0)} W
            </div>
          </div>

          {/* Time */}
          <div style={{ color: 'white', fontFamily: 'monospace', fontSize: '1.25rem' }}>
            t = {currentTime.toFixed(2)}s
          </div>

        </div>
      </section>

      {/* 2. LIVE KPI CARDS */}
      <section className="metrics-grid">
        <MetricCard 
          title="Body Displacement (Ours)" 
          value={Math.abs((currentFrame.z_c || 0) * 1000).toFixed(2)} 
          unit="mm" 
          icon={Activity} 
        />
        <MetricCard 
          title="Body Displacement (Base)" 
          value={Math.abs((currentFrame.base_z_c || 0) * 1000).toFixed(2)} 
          unit="mm" 
          icon={Car} 
        />
        <MetricCard 
          title="Body Pitch" 
          value={Math.abs(currentFrame.theta_mrad || 0).toFixed(2)} 
          unit="mrad" 
          icon={Gauge} 
        />
        <MetricCard 
          title="48V Aux Battery" 
          value={currentFrame.net_battery_kwh ? currentFrame.net_battery_kwh.toFixed(4) : "1.5000"} 
          unit="kWh" 
          icon={Battery} 
        />
        <MetricCard 
          title="Battery SoC" 
          value={((currentFrame.battery_soc || 0.5) * 100).toFixed(1)} 
          unit="%" 
          icon={BatteryCharging}
          highlight={currentFrame.battery_soc > 0.5}
        />
        <MetricCard 
          title="Range Saved" 
          value={currentFrame.harvested_range_m ? (currentFrame.harvested_range_m / 1000).toFixed(4) : "0.0000"} 
          unit="km" 
          icon={Zap} 
          highlight={currentFrame.harvested_range_m > 0}
        />
        {/* Instant Power Card */}
        <div className={`metric-card ${currentFrame.power_conditioned_w > 100 ? 'highlight' : ''}`}>
          <div className="metric-header">
            <h3 className="metric-title">Conditioned Power</h3>
            <TrendingUp className="metric-icon" size={20} color={currentFrame.power_conditioned_w > 100 ? '#10b981' : 'var(--text-secondary)'} />
          </div>
          <div className="metric-value-container">
            <span className="metric-value" style={{ color: currentFrame.power_conditioned_w > 100 ? '#10b981' : 'var(--text-primary)'}}>
              {currentFrame.power_conditioned_w > 0 ? `+${(currentFrame.power_conditioned_w || 0).toFixed(0)}` : '0'}
            </span>
            <span className="metric-unit">W</span>
          </div>
        </div>
      </section>

      {/* 3. CHARTS GRID — Live Time-Series */}
      <section className="charts-grid">
        {/* Panel 1: Road Profile */}
        <ChartPanel colSpan={2} title="1. Road Profile (Mixed Pothole Track)" subtitle="Front wheel (dark) + Rear wheel (light, delayed by wheelbase/velocity)">
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
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Area type="monotone" dataKey="w_f_mm" name="Front Road (mm)" stroke="#64748b" strokeWidth={1.5} fillOpacity={1} fill="url(#colorRoad)" isAnimationActive={false} />
              <Area type="monotone" dataKey="w_r_mm" name="Rear Road (mm)" stroke="#94a3b8" strokeWidth={0.8} fillOpacity={0} isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </AreaChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 2: UKF Estimator */}
        <ChartPanel title="2. UKF Road Estimation (Novelty 1A)" subtitle="Real-time severity ρ and rate-of-change ρ̇ for feedforward">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Line type="stepAfter" dataKey="rho_f" name="ρ (Severity)" stroke="#ef4444" strokeWidth={2} dot={false} isAnimationActive={false} />
              <Line type="monotone" dataKey="rho_dot_scaled" name="0.1 × ρ̇ (Rate)" stroke="#f59e0b" strokeWidth={2} strokeDasharray="5 5" dot={false} isAnimationActive={false} />
              <ReferenceLine y={0.7} stroke="#ef4444" strokeDasharray="3 3" label={{ value: 'Danger', fill: '#ef4444', fontSize: 11 }} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </LineChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 3: LPV Actuator Force */}
        <ChartPanel title="3. LPV Actuator Force (Novelty 1B)" subtitle="Adaptive controller force — front (purple) + rear (light)">
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
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Area type="monotone" dataKey="u_f" name="Front u_f (N)" stroke="#8b5cf6" strokeWidth={2} fillOpacity={1} fill="url(#colorForce)" isAnimationActive={false} />
              <Area type="monotone" dataKey="u_r" name="Rear u_r (N)" stroke="#a78bfa" strokeWidth={1} fillOpacity={0} isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </AreaChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 4: Digital Twin */}
        <ChartPanel colSpan={2} title="4. Digital Twin: Real-Time Suspension Kinematics" subtitle="AI actively absorbing road severity while holding the cabin steady">
          <div style={{ width: '100%', height: '100%', minHeight: '300px' }}>
            <CarVisualizer currentFrame={currentFrame} data={data} currentTime={currentTime} />
          </div>
        </ChartPanel>

        {/* Panel 5: Body Acceleration Comparison */}
        <ChartPanel colSpan={2} title="5. Ride Comfort — Body Acceleration (Lower = Better)" subtitle="Adaptive LPV (blue) vs Fixed H∞ Base Paper (grey)">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" tickFormatter={(val) => `${val.toFixed(1)}s`} />
              <YAxis stroke="#94a3b8" unit=" m/s²" />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Line type="monotone" dataKey="base_z_c_ddot" name="Base Paper (Fixed H∞)" stroke="#7f8c8d" strokeWidth={1.2} dot={false} isAnimationActive={false} />
              <Line type="monotone" dataKey="z_c_ddot" name="Our Project (Adaptive LPV)" stroke="#3b82f6" strokeWidth={2} dot={false} isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </LineChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 6: Body Pitch Comparison */}
        <ChartPanel title="6. Angular Comfort — Body Pitch" subtitle="Pitch angle in milliradians (lower = less nausea)">
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" unit=" mrad" />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Line type="monotone" dataKey="base_theta_mrad" name="Base Paper" stroke="#7f8c8d" strokeWidth={1.2} dot={false} isAnimationActive={false} />
              <Line type="monotone" dataKey="theta_mrad" name="Our Project" stroke="#3b82f6" strokeWidth={2} dot={false} isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </LineChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 7: Power Electronics */}
        <ChartPanel title="7. Power Electronics — LC Filter Output" subtitle="Raw regen (orange) vs LC-conditioned clean DC (green)">
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <defs>
                <linearGradient id="colorConditioned" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#10b981" stopOpacity={0.4}/>
                  <stop offset="95%" stopColor="#10b981" stopOpacity={0}/>
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" unit=" W" />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Area type="monotone" dataKey="power_raw_w" name="Raw Regen (W)" stroke="#f97316" strokeWidth={1} fillOpacity={0.1} fill="#f97316" isAnimationActive={false} />
              <Area type="monotone" dataKey="power_conditioned_w" name="LC-Conditioned (W)" stroke="#10b981" strokeWidth={2} fillOpacity={1} fill="url(#colorConditioned)" isAnimationActive={false} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </AreaChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 8: Energy Balance */}
        <ChartPanel title="8. Energy Balance — Harvested vs Consumed" subtitle="Green = harvested in ECO mode, Red = consumed in COMFORT mode">
          <ResponsiveContainer width="100%" height="100%">
            <ComposedChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" />
              <YAxis stroke="#94a3b8" unit=" kJ" />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Area type="monotone" dataKey="cum_harvested_kj" name="Cumulative Harvested (kJ)" stroke="#10b981" strokeWidth={2} fillOpacity={0.2} fill="#10b981" isAnimationActive={false} />
              <Area type="monotone" dataKey="cum_consumed_kj_neg" name="Cumulative Consumed (kJ)" stroke="#ef4444" strokeWidth={2} fillOpacity={0.2} fill="#ef4444" isAnimationActive={false} />
              <Line type="monotone" dataKey="net_energy_kj" name="Net Balance (kJ)" stroke="#fbbf24" strokeWidth={2.5} strokeDasharray="5 5" dot={false} isAnimationActive={false} />
              <ReferenceLine y={0} stroke="rgba(255,255,255,0.2)" />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </ComposedChart>
          </ResponsiveContainer>
        </ChartPanel>

        {/* Panel 9: Battery SoC with Mode Overlay */}
        <ChartPanel colSpan={2} title="9. TD3 DRL Battery Management — SoC with COMFORT/ECO Mode" subtitle="Yellow line = Battery %, Green regions = ECO harvesting, Blue regions = COMFORT (active pushing)">
          <ResponsiveContainer width="100%" height="100%">
            <ComposedChart data={chartData} margin={{ top: 10, right: 30, left: 0, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="rgba(255,255,255,0.05)" vertical={false} />
              <XAxis dataKey="time" stroke="#94a3b8" tickFormatter={(val) => `${val.toFixed(1)}s`} />
              <YAxis yAxisId="soc" stroke="#eab308" unit="%" domain={['auto', 'auto']} />
              <YAxis yAxisId="mode" orientation="right" stroke="#10b981" domain={[0, 1]} hide />
              <Tooltip contentStyle={{ background: 'rgba(10,14,23,0.95)', border: '1px solid rgba(255,255,255,0.1)', borderRadius: '8px' }} />
              <Legend />
              <Bar yAxisId="mode" dataKey="mode_f" name="ECO Mode Active" fill="#10b981" fillOpacity={0.15} isAnimationActive={false} />
              <Line yAxisId="soc" type="monotone" dataKey="battery_soc" name="Battery SoC" stroke="#eab308" strokeWidth={3} dot={false} isAnimationActive={false} 
                tickFormatter={(val) => `${(val * 100).toFixed(0)}%`} />
              <ReferenceLine yAxisId="soc" y={0.15} stroke="#ef4444" strokeDasharray="3 3" label={{ value: 'Critical 15%', fill: '#ef4444', fontSize: 11 }} />
              <ReferenceLine x={currentTime} stroke="#00F0FF" strokeWidth={2} />
            </ComposedChart>
          </ResponsiveContainer>
        </ChartPanel>
      </section>

      {/* 4. STATIC ANALYSIS SECTION */}
      <section className="charts-grid" style={{ marginTop: '1.5rem' }}>
        <ChartPanel title="10. Mechanical Damping vs Active Control" subtitle="Bode Plot — Frequency Domain Analysis">
          <div style={{ width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
            <img 
              src="/bode_plot.png" 
              alt="Bode Plot" 
              style={{ maxWidth: '100%', maxHeight: '400px', objectFit: 'contain', borderRadius: '8px' }} 
            />
          </div>
        </ChartPanel>

        <ChartPanel title="11. Regenerative Power Stabilization" subtitle="LC Filter + Shift Transformer (9.00 kg added mass)">
          <div style={{ width: '100%', height: '100%', display: 'flex', justifyContent: 'center', alignItems: 'center' }}>
            <img 
              src="/power_stability_plot.png" 
              alt="Power Stability Plot" 
              style={{ maxWidth: '100%', maxHeight: '400px', objectFit: 'contain', borderRadius: '8px' }} 
            />
          </div>
        </ChartPanel>

        <ChartPanel colSpan={2} title="12. Control System Stability — Pole-Zero Map" subtitle="All closed-loop poles in LHP confirms BIBO stability">
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
