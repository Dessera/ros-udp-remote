import { createApp } from "vue";
import "./styles.css";
import App from "./App.vue";

const app = createApp(App);

app.directive("draggable", {
  mounted(el, binding) {
    el.style.position = "absolute";
    el.style.cursor = "move";

    const { value } = binding;

    let startX = 0;
    let startY = 0;
    let initialMouseX = 0;
    let initialMouseY = 0;

    const mouseDownHandler = (e: any) => {
      startX = el.offsetLeft;
      startY = el.offsetTop;
      initialMouseX = e.clientX;
      initialMouseY = e.clientY;

      document.addEventListener("mousemove", mouseMoveHandler);
      document.addEventListener("mouseup", mouseUpHandler);
    };

    const mouseMoveHandler = (e: any) => {
      const dx = e.clientX - initialMouseX;
      const dy = e.clientY - initialMouseY;

      el.style.left = `${startX + dx}px`;
      el.style.top = `${startY + dy}px`;
    };

    const mouseUpHandler = () => {
      document.removeEventListener("mousemove", mouseMoveHandler);
      document.removeEventListener("mouseup", mouseUpHandler);

      // reset the position
      el.style.left = `${startX}px`;
      el.style.top = `${startY}px`;
    };

    el.addEventListener("mousedown", mouseDownHandler);
  }
});

app.mount("#app");
