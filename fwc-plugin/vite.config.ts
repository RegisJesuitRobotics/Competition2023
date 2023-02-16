import { defineConfig } from "vite";

// https://vitejs.dev/config/
export default defineConfig({
  build: {
    lib: {
      name: "raider-plugin",
      entry: "src/plugin.ts",
      formats: ["es"],
      fileName: "index",
    },
    outDir: "dist",
  },
  server: {
    open: "/",
  },
});
