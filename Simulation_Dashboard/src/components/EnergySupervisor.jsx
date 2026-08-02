import React, { useEffect, useState } from 'react';

export default function EnergySupervisor({ isEcoMode, setIsEcoMode }) {
  const [soc, setSoc] = useState(25.0); // Start at 25% battery
  const [power, setPower] = useState(0);

  useEffect(() => {
    const drainInterval = setInterval(() => {
      setSoc(prev => {
        const newSoc = prev - 0.2;
        if (newSoc <= 15.0 && !isEcoMode) {
          setIsEcoMode(true);
        }
        if (newSoc <= 0) return 0;
        return newSoc;
      });
    }, 500);

    return () => clearInterval(drainInterval);
  }, [isEcoMode, setIsEcoMode]);

  useEffect(() => {
    if (isEcoMode) {
      // Simulate power generation spikes
      const powerInterval = setInterval(() => {
        setPower(Math.floor(Math.random() * 500) + 1200); // 1200 - 1700 W
        setTimeout(() => setPower(0), 200);
      }, 1500);
      return () => clearInterval(powerInterval);
    } else {
      setPower(0);
    }
  }, [isEcoMode]);

  return (
    <div className="glass-panel">
      <h2>Layer 3: DRL Energy Agent</h2>
      
      <div style={{ flexGrow: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
        <div className={`status-indicator ${isEcoMode ? 'energy' : 'safe'}`}>
          <div className="status-dot"></div>
          {isEcoMode ? 'ECO REGENERATION MODE' : 'COMFORT MODE'}
        </div>
        
        <div style={{ margin: '30px 0' }}>
          <p>Battery State of Charge (SoC):</p>
          <div className="data-readout" style={{ color: soc <= 15 ? 'var(--neon-red)' : 'white' }}>
            {soc.toFixed(1)}%
          </div>
          <div className="meter-container">
            <div 
              className="meter-fill" 
              style={{ width: `${soc}%` }}
            ></div>
          </div>
        </div>

        <div>
          <p>Faraday Induction Harvested:</p>
          <div className="data-readout" style={{ color: 'var(--neon-green)' }}>
            +{power} W
          </div>
        </div>
      </div>

      <button className="action-btn" onClick={() => {
        setSoc(25.0);
        setIsEcoMode(false);
      }}>
        Recharge Battery
      </button>
    </div>
  );
}
