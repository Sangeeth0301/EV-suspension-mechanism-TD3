import React from 'react';

const MetricCard = ({ title, value, unit, icon: Icon, highlight = false }) => {
  return (
    <div className={`metric-card ${highlight ? 'highlight' : ''}`}>
      <div className="metric-header">
        <h3 className="metric-title">{title}</h3>
        {Icon && <Icon className="metric-icon" size={20} />}
      </div>
      <div className="metric-value-container">
        <span className="metric-value">{value}</span>
        {unit && <span className="metric-unit">{unit}</span>}
      </div>
    </div>
  );
};

export default MetricCard;
