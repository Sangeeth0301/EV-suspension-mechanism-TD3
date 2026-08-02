import React from 'react';

const ChartPanel = ({ title, subtitle, children, colSpan = 1 }) => {
  return (
    <div className="chart-panel" style={{ gridColumn: `span ${colSpan}` }}>
      <div className="chart-header">
        <h3 className="chart-title">{title}</h3>
        {subtitle && <p className="chart-subtitle">{subtitle}</p>}
      </div>
      <div className="chart-content">
        {children}
      </div>
    </div>
  );
};

export default ChartPanel;
