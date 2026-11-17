import react from '@vitejs/plugin-react';
import { defineConfig } from 'vite';

/**
 * The modelling server's API for the development server; the modelling server
 * itself runs separately on its default port (npm run serve:starting-points). The
 * browser's Host header is kept, because the modelling server takes commands
 * only from pages whose Origin names the host the request was sent to; a plain
 * target string would make Vite rewrite Host to the target.
 */
export const modellingServerProxy = { target: 'http://127.0.0.1:6651', changeOrigin: false };

export default defineConfig({
  plugins: [react()],
  server: { port: 5183, strictPort: true, proxy: { '/api': modellingServerProxy } },
  // One chunk, most of it three.js, served from the local machine.
  build: { chunkSizeWarningLimit: 1200 },
});
