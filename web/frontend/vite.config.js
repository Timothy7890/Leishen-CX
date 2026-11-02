import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

// 构建产物输出到 web/dist，由 web/server.py 提供
// 本地开发：VITE_API=http://localhost:7010 npm run dev
export default defineConfig({
  plugins: [vue()],
  base: './',
  build: { outDir: '../dist', emptyOutDir: true },
  server: {
    host: true,
    proxy: { '/api': process.env.VITE_API || 'http://localhost:7010' },
  },
})
