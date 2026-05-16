const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const { spawn } = require('child_process');
const fs = require('fs');
const http = require('http');
const net = require('net');
const path = require('path');

let backendProcess = null;
let mainWindow = null;

ipcMain.handle('dialog:open-file', async (_event, options = {}) => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: options.title || 'Select file',
    defaultPath: options.currentPath || undefined,
    properties: ['openFile'],
    filters: options.filters || [{ name: 'All files', extensions: ['*'] }]
  });
  if (result.canceled || !result.filePaths.length) {
    return null;
  }
  return result.filePaths[0];
});

function loadingHtml(title, message) {
  return `<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <style>
    :root { color-scheme: light; font-family: Inter, Segoe UI, system-ui, sans-serif; }
    body { margin: 0; background: #f6f7f9; color: #172033; }
    .boot { align-items: center; display: flex; height: 100vh; justify-content: center; }
    .card { background: #fff; border: 1px solid #e3e7ee; border-radius: 10px; box-shadow: 0 18px 55px rgba(15,23,42,.08); padding: 28px; width: 420px; }
    .row { align-items: center; display: flex; gap: 14px; }
    .mark { align-items: center; background: #172033; border-radius: 9px; color: white; display: flex; font-weight: 800; height: 42px; justify-content: center; width: 42px; }
    h1 { font-size: 19px; margin: 0; }
    p { color: #64748b; line-height: 1.5; margin: 8px 0 0; }
    .bar { background: #edf1f6; border-radius: 999px; height: 7px; margin-top: 22px; overflow: hidden; }
    .bar span { animation: load 1.25s ease-in-out infinite; background: #1264d8; border-radius: inherit; display: block; height: 100%; width: 36%; }
    @keyframes load { 0% { transform: translateX(-100%); } 100% { transform: translateX(300%); } }
  </style>
</head>
<body>
  <main class="boot">
    <section class="card">
      <div class="row">
        <div class="mark">B</div>
        <div>
          <h1>${title}</h1>
          <p>${message}</p>
        </div>
      </div>
      <div class="bar"><span></span></div>
    </section>
  </main>
</body>
</html>`;
}

function log(message) {
  try {
    const dir = path.join(process.env.LOCALAPPDATA || app.getPath('userData'), 'BuckPidAutoTuner');
    fs.mkdirSync(dir, { recursive: true });
    fs.appendFileSync(path.join(dir, 'electron.log'), `[${new Date().toISOString()}] ${message}\n`);
  } catch (_) {
    // Logging must never prevent the app from starting.
  }
}

function findFreePort() {
  return new Promise((resolve, reject) => {
    const server = net.createServer();
    server.unref();
    server.on('error', reject);
    server.listen(0, '127.0.0.1', () => {
      const address = server.address();
      const port = typeof address === 'object' && address ? address.port : 8765;
      server.close(() => resolve(port));
    });
  });
}

function backendCommand(port) {
  if (app.isPackaged) {
    return {
      command: path.join(process.resourcesPath, 'backend', 'buck_web_backend.exe'),
      args: ['--host', '127.0.0.1', '--port', String(port)]
    };
  }
  const root = path.resolve(__dirname, '..', '..');
  return {
    command: process.env.PYTHON || 'python',
    args: [path.join(root, 'web_backend.py'), '--host', '127.0.0.1', '--port', String(port)]
  };
}

function waitForBackend(port, timeoutMs = 20000) {
  const started = Date.now();
  return new Promise((resolve, reject) => {
    const poll = () => {
      const req = http.get(`http://127.0.0.1:${port}/api/health`, (res) => {
        res.resume();
        if (res.statusCode === 200) {
          resolve();
        } else {
          retry();
        }
      });
      req.on('error', retry);
      req.setTimeout(1200, () => {
        req.destroy();
        retry();
      });
    };
    const retry = () => {
      if (Date.now() - started > timeoutMs) {
        reject(new Error('Backend did not become ready in time.'));
        return;
      }
      setTimeout(poll, 300);
    };
    poll();
  });
}

async function startBackend() {
  const port = await findFreePort();
  const { command, args } = backendCommand(port);
  const externalResourceRoot = app.isPackaged ? path.dirname(process.execPath) : path.resolve(__dirname, '..', '..');
  log(`Starting backend: ${command} ${args.join(' ')}`);
  log(`Backend exists: ${fs.existsSync(command)}`);
  log(`Default model resource root: ${externalResourceRoot}`);
  backendProcess = spawn(command, args, {
    cwd: path.resolve(__dirname, '..', '..'),
    env: {
      ...process.env,
      BUCK_AUTOTUNER_RESOURCE_ROOT: externalResourceRoot
    },
    windowsHide: true,
    stdio: ['ignore', 'pipe', 'pipe']
  });
  backendProcess.stdout.on('data', (data) => log(`backend stdout: ${String(data).trim()}`));
  backendProcess.stderr.on('data', (data) => log(`backend stderr: ${String(data).trim()}`));
  backendProcess.on('error', (error) => log(`backend spawn error: ${error.message}`));
  backendProcess.on('exit', () => {
    log('backend exited');
    backendProcess = null;
  });
  await waitForBackend(port);
  log(`Backend ready on ${port}`);
  return `http://127.0.0.1:${port}`;
}

async function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1500,
    height: 940,
    minWidth: 1180,
    minHeight: 760,
    backgroundColor: '#f6f7f9',
    title: 'Buck PID Auto-Tuner',
    show: false,
    webPreferences: {
      contextIsolation: true,
      nodeIntegration: false,
      preload: path.join(__dirname, 'preload.cjs')
    }
  });
  mainWindow.maximize();
  await mainWindow.loadURL(
    `data:text/html;charset=utf-8,${encodeURIComponent(
      loadingHtml('Starting Buck PID Auto-Tuner', 'Preparing the local simulation backend. This can take a few seconds in the portable EXE.')
    )}`
  );
  mainWindow.show();

  const apiBase = await startBackend();
  const query = `?api=${encodeURIComponent(apiBase)}`;
  if (!app.isPackaged && process.env.ELECTRON_RENDERER_URL) {
    log(`Loading dev renderer: ${process.env.ELECTRON_RENDERER_URL}`);
    await mainWindow.loadURL(`${process.env.ELECTRON_RENDERER_URL}${query}`);
  } else {
    log(`Loading packaged renderer from ${path.join(__dirname, '..', 'dist', 'index.html')}`);
    await mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'), { query: { api: apiBase } });
  }
}

app.whenReady().then(createWindow).catch((error) => {
  log(`startup failed: ${error.stack || error.message}`);
  dialog.showErrorBox('Buck PID Auto-Tuner failed to start', error.message);
  app.quit();
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});

app.on('before-quit', () => {
  if (backendProcess) {
    backendProcess.kill();
    backendProcess = null;
  }
});
