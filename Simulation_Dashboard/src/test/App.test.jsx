import React from 'react';
import { render, screen, waitFor } from '@testing-library/react';
import App from '../App';

// Mock the fetch API to return dummy data for our charts
global.fetch = vi.fn(() =>
  Promise.resolve({
    json: () => Promise.resolve([
      { time: 0, w_f: 0.01, rho_f: 0, rho_dot_f: 0, u_f: 100, base_z_c: 0, z_c: 0, z_uf: 0, battery_soc: 1.0, energy_harvested_j: 0, instant_power_w: 0, range_added_m: 0, cyber_attack: false, net_battery_kwh: 50.0, remaining_range_km: 333.33, harvested_range_m: 0 },
      { time: 1, w_f: -0.01, rho_f: 0.5, rho_dot_f: 0.2, u_f: -100, base_z_c: 0.1, z_c: 0.05, z_uf: -0.01, battery_soc: 0.99, energy_harvested_j: 10, instant_power_w: 1200, range_added_m: 0.02, cyber_attack: true, net_battery_kwh: 49.99, remaining_range_km: 333.32, harvested_range_m: 0.02 }
    ]),
  })
);

describe('App Dashboard', () => {
  it('renders loading state initially', () => {
    render(<App />);
    expect(screen.getByText(/Loading Simulation Data.../i)).toBeInTheDocument();
  });

  it('renders the dashboard after fetching data', async () => {
    render(<App />);
    
    // Wait for the data to load and the loading screen to disappear
    await waitFor(() => {
      expect(screen.queryByText(/Loading Simulation Data.../i)).not.toBeInTheDocument();
    });

    // Check header
    expect(screen.getByText(/Cyber-Resilient Active Suspension/i)).toBeInTheDocument();
    
    // Check all metric cards
    expect(screen.getByText('Comfort (RMS Accel)')).toBeInTheDocument();
    expect(screen.getByText('Base Comfort (No AI)')).toBeInTheDocument();
    expect(screen.getByText('48V Aux Battery')).toBeInTheDocument();
    expect(screen.getByText('Remaining Range')).toBeInTheDocument();
    expect(screen.getByText('Range Saved by Suspension')).toBeInTheDocument();
    expect(screen.getByText('Instant Power')).toBeInTheDocument();

    // Check all chart panels (we now have a digital twin visualizer instead of panel 4)
    expect(screen.getByText(/1. Road Profile/i)).toBeInTheDocument();
    expect(screen.getByText(/2. UKF Road Estimation/i)).toBeInTheDocument();
    expect(screen.getByText(/3. LPV Actuator Force/i)).toBeInTheDocument();
    expect(screen.getByText(/4. Digital Twin/i)).toBeInTheDocument();
  });
});
