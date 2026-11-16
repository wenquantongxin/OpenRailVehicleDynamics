import { StrictMode } from 'react';
import { createRoot } from 'react-dom/client';

import './vehicle_modeling_interface_styles.css';
import { VehicleModelingInterface } from './VehicleModelingInterface.tsx';

const container = document.getElementById('root');
if (container === null) {
  throw new Error('index.html has no #root element');
}
createRoot(container).render(
  <StrictMode>
    <VehicleModelingInterface />
  </StrictMode>,
);
