import { html, css, LitElement } from "lit";
import { customElement, property } from "lit/decorators.js";

@customElement("grid-selector")
export class GridSelector extends LitElement {
  static styles = css`
    .full {
      width: 100%;
      height: 100%;
    }

    .section {
      display: inline-block;
      box-sizing: border-box;
      width: 11.11%;
      height: 100%;
      border-style: solid;
      border-width: 15px;
    }
  `;

  @property({ type: Number, reflect: true })
  selected = 0;

  render() {
    const itemTemplates = [];
    for (let i = 0; i < 9; i++) {
      itemTemplates.push(
        html`<div
          class="section"
          style="background-color:${[1, 4, 7].includes(i)
            ? "purple"
            : "yellow"}; border-color:${i == this.selected ? "lime" : "black"}"
          @click=${() => (this.selected = i)}
        ></div>`
      );
    }

    return html`<div class="full">${itemTemplates}</div>`;
  }
}
