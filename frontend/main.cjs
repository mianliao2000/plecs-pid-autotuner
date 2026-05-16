const { app, BrowserWindow, dialog } = require('electron');
const { spawn } = require('child_process');
const fs = require('fs');
const http = require('http');
const net = require('net');
const path = require('path');

let backendProcess = null;
let mainWindow = null;

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
  log(`Starting backend: ${command} ${args.join(' ')}`);
  log(`Backend exists: ${fs.existsSync(command)}`);
  backendProcess = spawn(command, args, {
    cwd: path.resolve(__dirname, '..', '..'),
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
  const apiBase = await startBackend();
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
      nodeIntegration: false
    }
  });

  const query = `?api=${encodeURIComponent(apiBase)}`;
  if (!app.isPackaged && process.env.ELECTRON_RENDERER_URL) {
    log(`Loading dev renderer: ${process.env.ELECTRON_RENDERER_URL}`);
    await mainWindow.loadURL(`${process.env.ELECTRON_RENDERER_URL}${query}`);
  } else {
    log(`Loading packaged renderer from ${path.join(__dirname, '..', 'dist', 'index.html')}`);
    await mainWindow.loadFile(path.join(__dirname, '..', 'dist', 'index.html'), { query: { api: apiBase } });
  }
  mainWindow.maximize();
  mainWindow.show();
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
