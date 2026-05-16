const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('buckAutoTuner', {
  openFile: (options) => ipcRenderer.invoke('dialog:open-file', options)
});
