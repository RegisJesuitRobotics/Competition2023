import "./grid-selector";
import "./alerts.ts";
import { FrcDashboard } from "@frc-web-components/fwc";

export default function addPlugin(dashboard: FrcDashboard) {
  dashboard.addElements(
    {
      "grid-selector": {
        dashboard: {
          displayName: "Grid Selector",
        },
        properties: {
          selected: { type: "Number", reflect: true, primary: true },
        },
      },
      "network-alerts": {
        dashboard: {
          displayName: "Alerts",
        },
        properties: {
          errors: { type: "Array", reflect: true },
          warnings: { type: "Array", reflect: true },
          infos: { type: "Array", reflect: true },
        },
      },
    },
    "RaiderPlugin"
  );
}
