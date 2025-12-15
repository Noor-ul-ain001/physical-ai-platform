import React from 'react';
import clsx from 'clsx';
import styles from './HardwareTable.module.css';

type HardwareSpec = {
  component: string;
  minimum: string;
  recommended: string;
  notes: string;
};

type HardwareTableProps = {
  specs: HardwareSpec[];
};

const HARDWARE_SPECS: HardwareSpec[] = [
  {
    component: 'CPU',
    minimum: '4 cores',
    recommended: '8+ cores',
    notes: 'For simulation and AI processing'
  },
  {
    component: 'RAM',
    minimum: '8 GB',
    recommended: '16+ GB',
    notes: 'Essential for running simulations'
  },
  {
    component: 'GPU',
    minimum: 'Integrated',
    recommended: 'RTX 3060+',
    notes: 'Required for Isaac Sim and AI training'
  },
  {
    component: 'Storage',
    minimum: '50 GB',
    recommended: '100+ GB',
    notes: 'For ROS packages and datasets'
  },
  {
    component: 'Network',
    minimum: 'Gigabit Ethernet',
    recommended: '2.5 GbE+',
    notes: 'For robot communication'
  }
];

export default function HardwareTable({ specs = HARDWARE_SPECS }: HardwareTableProps): JSX.Element {
  return (
    <div className={clsx('container', styles.hardwareTableContainer)}>
      <h3>Hardware Specifications</h3>
      <table className={clsx('hardware-table', styles.hardwareTable)}>
        <thead>
          <tr>
            <th>Component</th>
            <th>Minimum</th>
            <th>Recommended</th>
            <th>Notes</th>
          </tr>
        </thead>
        <tbody>
          {specs.map((spec, index) => (
            <tr key={index}>
              <td>{spec.component}</td>
              <td>{spec.minimum}</td>
              <td>{spec.recommended}</td>
              <td>{spec.notes}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}