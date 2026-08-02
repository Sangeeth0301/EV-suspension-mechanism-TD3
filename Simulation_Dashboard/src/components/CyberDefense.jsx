import React, { useState, useEffect } from 'react';

export default function CyberDefense({ isUnderAttack, setIsUnderAttack }) {
  const [networkPath, setNetworkPath] = useState('CAN BUS (500 kbps)');
  const [dropRate, setDropRate] = useState(0);

  useEffect(() => {
    let interval;
    if (isUnderAttack) {
      // Simulate DoS attack ramping up
      interval = setInterval(() => {
        setDropRate(prev => {
          if (prev >= 40) {
            setNetworkPath('AUTOMOTIVE ETHERNET (100BASE-T1)');
            return 0; // Failover successful
          }
          return prev + 5;
        });
      }, 100);
    } else {
      setNetworkPath('CAN BUS (500 kbps)');
      setDropRate(0);
    }
    return () => clearInterval(interval);
  }, [isUnderAttack]);

  const isSafe = networkPath.includes('ETHERNET') || (!isUnderAttack && dropRate === 0);

  return (
    <div className="glass-panel">
      <h2>Layer 2: Edge ML Defense</h2>
      
      <div style={{ flexGrow: 1, display: 'flex', flexDirection: 'column', justifyContent: 'center' }}>
        <div className={`status-indicator ${isSafe ? 'safe' : 'danger'}`}>
          <div className="status-dot"></div>
          {isSafe ? 'NETWORK SECURE' : 'DoS ATTACK DETECTED!'}
        </div>
        
        <div style={{ margin: '30px 0' }}>
          <p>Active Data Path:</p>
          <h3 style={{ color: networkPath.includes('ETHERNET') ? '#00ffcc' : 'white' }}>
            {networkPath}
          </h3>
        </div>

        <div>
          <p>Packet Drop Rate: {dropRate}%</p>
          <div className="meter-container">
            <div 
              className="meter-fill" 
              style={{ 
                width: `${dropRate}%`, 
                background: 'var(--neon-red)' 
              }}
            ></div>
          </div>
        </div>
      </div>

      <button 
        className={`action-btn ${isUnderAttack ? 'safe' : 'danger'}`} 
        onClick={() => setIsUnderAttack(!isUnderAttack)}
      >
        {isUnderAttack ? 'Stop Attack / Reset' : 'Launch DoS Flood'}
      </button>
    </div>
  );
}
