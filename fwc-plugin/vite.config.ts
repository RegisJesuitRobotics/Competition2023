import { defineConfig } from "vite";

// https://vitejs.dev/config/
export default defineConfig({
  build: {
    lib: {
      name: "my-plugin",
      entry: "src/plugin.ts",
      formats: ["es"],
      fileName: "index",
    },
    outDir: "raider-plugin",
  },
  server: {
    open: "/",
  },
});
