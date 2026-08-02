import React from 'react';
import { render, screen } from '@testing-library/react';
import MetricCard from './MetricCard';
import { Shield } from 'lucide-react';

describe('MetricCard', () => {
  it('renders the title and value correctly', () => {
    render(<MetricCard title="Bandwidth Saved" value="36.7" unit="%" />);
    
    expect(screen.getByText('Bandwidth Saved')).toBeInTheDocument();
    expect(screen.getByText('36.7')).toBeInTheDocument();
    expect(screen.getByText('%')).toBeInTheDocument();
  });

  it('renders the icon if provided', () => {
    const { container } = render(<MetricCard title="Test" value="10" icon={Shield} />);
    
    // Check if the svg from lucide-react is rendered
    expect(container.querySelector('svg')).toBeInTheDocument();
  });

  it('applies highlight class when highlight prop is true', () => {
    const { container } = render(<MetricCard title="Test" value="10" highlight={true} />);
    
    // Check if the main div has the 'highlight' class
    expect(container.firstChild).toHaveClass('highlight');
  });
});
