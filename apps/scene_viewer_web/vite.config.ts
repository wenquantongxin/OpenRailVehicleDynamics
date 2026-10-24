import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';

export default defineConfig({
  plugins: [react()],
  server: { proxy: { '/live': { target: 'http://127.0.0.1:6641', ws: true, changeOrigin: false }, '/bundles': 'http://127.0.0.1:6641' } },
});
