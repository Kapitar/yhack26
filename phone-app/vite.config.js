import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

export default defineConfig({
  plugins: [react()],
  server: {
    host: true, // expose on LAN so phone can connect during dev
    proxy: {
      '/valhalla': {
        target: 'http://vpn.artem-kim.com:8002',
        changeOrigin: true,
        rewrite: (path) => path.replace(/^\/valhalla/, ''),
      },
    },
  },
});