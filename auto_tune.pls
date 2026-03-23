// auto_tune.pls - Trial-and-Error PID Tuning with Full Data Logging
//
// Goals: Overshoot < 5%, Undershoot < 5%, NO oscillation
// Output: Every iteration saved to CSV for animation
//
// Usage: In PLECS, go to Simulation > Scripts > Run Script

double simTime = 0.01;  // 10ms
double loadStepTime = 0.005;  // 5ms

// Targets
double targetOvershoot = 5.0;
double targetUndershoot = 5.0;
int maxOscillations = 2;  // Max 2 crossings past Vsteady during settling

// Starting point: current parameters in model
double Kp, Ki, Kd, Kf;

// Results directory
string resultsDir = "results";

// Initialize
void init() {
    // Create results directory using system command
    system("mkdir results 2>nul");

    // Open log file
    file = fopen("results/tuning_log.csv", "w");
    fprintf(file, "Iter,Kp,Ki,Kd,Kf,Overshoot,Undershoot,OscCount,SettlingTime,Status\n");
    fclose(file);
}

void setParams() {
    plecs('set', 'synchronous buck/Voltage Compensator', 'Kp', Kp);
    plecs('set', 'synchronous buck/Voltage Compensator', 'Ki', Ki);
    plecs('set', 'synchronous buck/Voltage Compensator', 'Kd', Kd);
    plecs('set', 'synchronous buck/Voltage Compensator', 'Kf', Kf);
}

void runSim(int iter) {
    plecs('simulate', 't', simTime);
    // Export scope data to CSV with iteration number
    plecs('scope', 'export', 'Scope',
          sprintf("results/iter_%03d.csv", iter));
}

// Read CSV and return array of values
// This is a placeholder - actual implementation needs CSV parsing
double readCsvValue(string filename, string column, double time) {
    return 5.0;  // Placeholder
}

// Analyze response from saved CSV
// Returns overshoot, undershoot, oscillation count, settling time
void analyzeResponse(int iter, &double overshoot, &double undershoot,
                     &int oscCount, &double settlingTime) {
    // Parse the CSV file saved by scope export
    // CSV format: time,value pairs (first column is input, second is output)

    string filename = sprintf("results/iter_%03d.csv", iter);

    // Use PLECS scope data directly via plecs('scope', 'data', ...)
    // Scope name is "Scope" - need to check actual scope name in model

    // Placeholder values - in actual implementation, parse the CSV
    overshoot = 0.0;
    undershoot = 0.0;
    oscCount = 0;
    settlingTime = 0.0;
}

void logResult(int iter, double overshoot, double undershoot,
               int oscCount, double settlingTime, string status) {
    file = fopen("results/tuning_log.csv", "a");
    fprintf(file, "%d,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%d,%.6f,%s\n",
            iter, Kp, Ki, Kd, Kf,
            overshoot, undershoot, oscCount,
            settlingTime, status);
    fclose(file);
}

// Decision rules: analyze response and adjust parameters
void adjustParams(double overshoot, double undershoot, int oscCount, double settlingTime) {
    // Heavy overshoot: Reduce Kp, increase Kd
    if (overshoot > 10) {
        Kp = Kp * 0.6;
        Kd = Kd * 1.5;
        Ki = Ki * 0.8;
    }
    // Moderate overshoot: Fine-tune
    else if (overshoot > 5) {
        Kp = Kp * 0.85;
        Kd = Kd * 1.15;
    }

    // Undershoot issue: Increase Kp, reduce Kd
    if (undershoot > 5) {
        Kp = Kp * 1.2;
        Kd = Kd * 0.7;
        Ki = Ki * 1.1;
    }

    // Too oscillatory: Increase damping
    if (oscCount > maxOscillations) {
        Kd = Kd * 1.3;
        Kf = Kf * 2;
        Kp = Kp * 0.95;
    }

    // Slow settling: Increase integral action
    if (settlingTime > 0.003) {
        Ki = Ki * 1.3;
        Kp = Kp * 1.1;
    }

    // Ensure minimum values to avoid numerical issues
    if (Kp < 0.01) Kp = 0.01;
    if (Ki < 0.001) Ki = 0.001;
    if (Kd < 0.0) Kd = 0.0;
    if (Kf < 0.0) Kf = 0.0;
}

void tune() {
    // Read initial parameters from model
    Kp = plecs('get', 'synchronous buck/Voltage Compensator', 'Kp');
    Ki = plecs('get', 'synchronous buck/Voltage Compensator', 'Ki');
    Kd = plecs('get', 'synchronous buck/Voltage Compensator', 'Kd');
    Kf = plecs('get', 'synchronous buck/Voltage Compensator', 'Kf');

    printf("Initial parameters: Kp=%.4f Ki=%.4f Kd=%.4f Kf=%.4f\n",
           Kp, Ki, Kd, Kf);

    init();

    int maxIter = 40;
    for (iter = 0; iter < maxIter; iter++) {
        setParams();
        runSim(iter);

        double overshoot, undershoot, settlingTime;
        int oscCount;
        analyzeResponse(iter, overshoot, undershoot, oscCount, settlingTime);

        string status;
        if (overshoot < targetOvershoot &&
            undershoot < targetUndershoot &&
            oscCount <= maxOscillations) {
            status = "PASS";
        } else {
            status = "FAIL";
        }

        logResult(iter, overshoot, undershoot, oscCount, settlingTime, status);

        printf("Iter %02d: Kp=%.4f Ki=%.4f Kd=%.4f Kf=%.4f OS=%.1f%% US=%.1f%% OSC=%d -> %s\n",
               iter, Kp, Ki, Kd, Kf, overshoot, undershoot, oscCount, status);

        if (strcmp(status, "PASS") == 0) {
            printf("\n*** SUCCESS! ***\n");
            printf("Final: Kp=%.4f Ki=%.4f Kd=%.4f Kf=%.4f\n", Kp, Ki, Kd, Kf);
            printf("Overshoot=%.2f%% Undershoot=%.2f%% Oscillations=%d\n",
                   overshoot, undershoot, oscCount);
            break;
        }

        adjustParams(overshoot, undershoot, oscCount, settlingTime);
    }

    printf("\nTuning complete. Check results/tuning_log.csv\n");
}

tune();
