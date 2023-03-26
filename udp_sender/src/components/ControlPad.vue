<script setup lang="ts">
import { computed, onMounted, ref } from 'vue';
import { invoke } from '@tauri-apps/api/tauri';

let ip = ref("");
let port = ref("");
let button_state = ref(false);

let timer: NodeJS.Timer;

const connect = async () => {
  if (button_state.value) {
    button_state.value = false;
    clearInterval(timer);
    await invoke("disconnect");
    return;
  }

  invoke("connect", { cfg: { ip: ip.value, port: port.value } }).then(() => {
    button_state.value = true;
    timer = setInterval(() => {
      const controller = document.querySelector(".controller") as HTMLElement;
      let offs_z = (proto_x - controller.offsetLeft) / 250;
      let offs_x = (proto_y - controller.offsetTop) / 250;

      invoke("send", { data: { x: offs_x, z: offs_z } });
    }, 50);
  }).catch((err) => {
    console.log(err);
  });
}

let proto_x = 0;
let proto_y = 0;
onMounted(() => {
  const controller = document.querySelector(".controller") as HTMLElement;
  proto_x = controller.offsetLeft;
  proto_y = controller.offsetTop;
});
</script>

<template>
  <input type="text" v-model="ip" placeholder="UDP IP Address">
  <input type="text" v-model="port" placeholder="Port">
  <button @click="connect">{{ button_state ? "disconnect" : "connect" }}</button>
  <div class="outside_back">
    <div class="controller" v-draggable></div>
  </div>
</template>

<style scoped>
.outside_back {
  --back_radius: 500px;
  width: var(--back_radius);
  height: var(--back_radius);
  background-color: #000;
  border-radius: var(--back_radius);
  opacity: 0.5;

  position: relative;
}

.controller {
  --controller_radius: 200px;
  width: var(--controller_radius);
  height: var(--controller_radius);
  background-color: #a85a00;
  border-radius: var(--controller_radius);
  position: relative;
  top: calc((var(--back_radius) - var(--controller_radius)) / 2);
  left: calc((var(--back_radius) - var(--controller_radius)) / 2);

  cursor: pointer;
}
</style>
