import React, { useState, useEffect } from 'react';
import '../index.css';

export default function VehicleVisualizer({ isBouncing, setIsBouncing, isEcoMode }) {
  useEffect(() => {
    if (isBouncing) {
      const timer = setTimeout(() => setIsBouncing(false), 500);
      return () => clearTimeout(timer);
    }
  }, [isBouncing, setIsBouncing]);

  return (
    <div className="glass-panel">
      <h2>Layer 1: LPV Physics</h2>
      <div className="vehicle-container">
        {/* Chassis */}
        <div className={`chassis ${isBouncing ? 'bounce-chassis' : ''}`}>
          EV Chassis
        </div>
        
        {/* Active Suspension Shock Absorber */}
        <div className="suspension-strut">
          <div 
            className={`suspension-coil ${isBouncing ? 'coil-compress' : ''}`}
            style={{ 
              backgroundImage: isEcoMode 
                ? 'repeating-linear-gradient(transparent, transparent 10px, #00ff00 10px, #00ff00 15px)' 
                : undefined
            }}
          ></div>
        </div>
        
        {/* IWM Wheel */}
        <div className={`wheel ${isBouncing ? 'bounce-wheel' : ''}`}>
          <div className="wheel-hub"></div>
        </div>
      </div>
      
      <p style={{ color: '#888', marginTop: '20px', textAlign: 'center' }}>
        Kalman Filter tracks road roughness. LPV Controller instantly adjusts damping to keep chassis stable while wheel takes the impact.
      </p>

      <button className="action-btn" onClick={() => setIsBouncing(true)}>
        Hit Pothole
      </button>
    </div>
  );
}
