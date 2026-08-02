import React from 'react';

const CarVisualizer = ({ currentFrame, data, currentTime }) => {
  // SVG ViewBox
  const width = 800;
  const height = 300;
  
  // Center coordinates
  const centerX = width / 2;
  const groundY = height / 2 + 50; 
  
  // Scaling factors for visual exaggeration (so we can actually see the movement)
  // Our simulation units are meters (e.g. 0.05m = 50mm). We'll multiply by 1500 to get pixels.
  const scaleY = 1500; 
  
  // Car body dimensions
  const carWidth = 160;
  const carHeight = 60;
  
  // Values from physics engine
  const carY = groundY - 100 - (currentFrame.z_c * scaleY);
  const wheelY = groundY - 20 - (currentFrame.z_uf * scaleY);
  const roadCurrentHeight = groundY - (currentFrame.w_f * scaleY);

  // Generate the road profile path for the last 1 second and next 1 second
  const timeWindow = 1.0;
  const visibleData = data.filter(d => d.time >= currentTime - timeWindow && d.time <= currentTime + timeWindow);
  
  let roadPath = '';
  if (visibleData.length > 0) {
    const startX = 0;
    const endX = width;
    const timeStart = currentTime - timeWindow;
    const timeEnd = currentTime + timeWindow;
    
    // Map data points to SVG coordinates
    const points = visibleData.map(d => {
      const x = ((d.time - timeStart) / (timeEnd - timeStart)) * width;
      const y = groundY - (d.w_f * scaleY);
      return `${x},${y}`;
    });
    
    roadPath = `M 0,${groundY} L ${points.join(' L ')} L ${width},${groundY} L ${width},${height} L 0,${height} Z`;
  }

  return (
    <div style={{ width: '100%', height: '100%', background: 'linear-gradient(180deg, #0a0e17 0%, #1e293b 100%)', borderRadius: '12px', overflow: 'hidden', position: 'relative' }}>
      <svg width="100%" height="100%" viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="xMidYMid slice">
        
        {/* Background Grid */}
        <g stroke="rgba(255,255,255,0.05)" strokeWidth="1">
          {[0, 50, 100, 150, 200, 250, 300].map(y => (
            <line key={y} x1="0" y1={y} x2={width} y2={y} />
          ))}
        </g>
        
        {/* Road Surface */}
        <path d={roadPath} fill="#334155" stroke="#94a3b8" strokeWidth="2" />
        
        {/* Spring/Damper Strut */}
        <line 
          x1={centerX} 
          y1={carY + carHeight/2} 
          x2={centerX} 
          y2={wheelY} 
          stroke="#3b82f6" 
          strokeWidth="6" 
          strokeDasharray="4 4"
        />
        
        {/* Unsprung Mass (Wheel) */}
        <circle 
          cx={centerX} 
          cy={wheelY} 
          r="24" 
          fill="#0f172a" 
          stroke={currentFrame.instant_power_w > 1000 ? "#10b981" : "#94a3b8"} 
          strokeWidth="4" 
        />
        {/* Hub */}
        <circle cx={centerX} cy={wheelY} r="8" fill="#cbd5e1" />
        
        {/* Sprung Mass (Car Body) */}
        <g transform={`translate(${centerX - carWidth/2}, ${carY - carHeight/2})`}>
          {/* Chassis */}
          <rect 
            width={carWidth} 
            height={carHeight} 
            rx="12" 
            fill="rgba(30, 41, 59, 0.9)" 
            stroke="#e2e8f0" 
            strokeWidth="2" 
          />
          {/* Windows */}
          <rect x="20" y="10" width="40" height="20" rx="4" fill="#3b82f6" fillOpacity="0.2" />
          <rect x="65" y="10" width="75" height="20" rx="4" fill="#3b82f6" fillOpacity="0.2" />
          {/* Headlight */}
          <circle cx="150" cy="40" r="5" fill="#fef08a" style={{ filter: 'drop-shadow(0 0 10px #fef08a)' }} />
          
          {/* Label */}
          <text x="20" y="50" fill="#fff" fontSize="12" fontWeight="bold">Sprung Mass</text>
        </g>
        
        {/* Dynamic Measurements overlay */}
        <g fill="#94a3b8" fontSize="13" fontFamily="Inter, sans-serif" fontWeight="500">
          <text x="20" y="30">Road Deflection: {(currentFrame.w_f * 1000).toFixed(1)} mm</text>
          <text x="20" y="55">Cabin Bounce: {(currentFrame.z_c * 1000).toFixed(1)} mm</text>
          <text x="20" y="80" fill={currentFrame.instant_power_w > 100 ? "#10b981" : "#8b5cf6"}>
            EM Motor Load: {currentFrame.u_f.toFixed(0)} N
          </text>
        </g>
        
      </svg>
    </div>
  );
};

export default CarVisualizer;
