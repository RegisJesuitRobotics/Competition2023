import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";
import "fa-icons";

@customElement("network-alerts")
export class Alerts extends LitElement {
  static styles = css`
    :host {
      color: var(--lumo-contrast, black);
    }
  `;

  @property({ type: Array, reflect: true })
  errors = [];
  @property({ type: Array, reflect: true })
  warnings = [];
  @property({ type: Array, reflect: true })
  infos = [];

  @property() colors = ["red", "green", "blue"];

  render() {
    console.log(this.errors);
    return html` <div>
      <h1 style="text-align: center">Alerts</h1>
      ${this.errors.length + this.warnings.length + this.infos.length == 0
        ? html`<p>Nothing to report</p>`
        : html`${this.errors.map(
            (error) => html` <p class="alert">
              <fa-icon class="fas fa-times-circle" color="red"></fa-icon>
              ${error}
            </p>`
          )}
          ${this.warnings.map(
            (warning) => html` <p class="alert">
              <fa-icon
                class="fas fa-exclamation-triangle"
                color="yellow"
              ></fa-icon>
              ${warning}
            </p>`
          )}
          ${this.infos.map(
            (info) => html` <p class="alert">
              <fa-icon class="fas fa-info-circle" color="green"></fa-icon>
              ${info}
            </p>`
          )} `}
    </div>`;
  }
}
