/*
 * Desktop unit-test runner for SignalQuality.c
 *
 * Compile (GCC):
 *   gcc -std=c11 -Wall -Wextra -O2 test_signal_quality.c SignalQuality.c -lm -o test_signal_quality
 *
 * Run from the folder containing ppg_quality_test_cases:
 *   ./test_signal_quality ppg_quality_test_cases
 *
 * On Windows (MinGW):
 *   gcc -std=c11 -Wall -Wextra -O2 test_signal_quality.c SignalQuality.c -lm -o test_signal_quality.exe
 *   .\test_signal_quality.exe ppg_quality_test_cases
 */
#include "SignalQuality.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>

#define MAX_TEST_SAMPLES 10000
#define MAX_TEST_BEATS   1000
#define MAX_ROW         1024
#define MAX_PATH_LEN    1024

static const char *status_name(SignalQualityStatus s)
{
    switch (s) {
        case SIGNAL_QUALITY_GOOD:                return "SIGNAL_QUALITY_GOOD";
        case SIGNAL_QUALITY_INVALID_INPUT:       return "SIGNAL_QUALITY_INVALID_INPUT";
        case SIGNAL_QUALITY_TOO_FEW_SAMPLES:     return "SIGNAL_QUALITY_TOO_FEW_SAMPLES";
        case SIGNAL_QUALITY_NONFINITE_DATA:      return "SIGNAL_QUALITY_NONFINITE_DATA";
        case SIGNAL_QUALITY_FLAT_SIGNAL:         return "SIGNAL_QUALITY_FLAT_SIGNAL";
        case SIGNAL_QUALITY_SATURATED:           return "SIGNAL_QUALITY_SATURATED";
        case SIGNAL_QUALITY_TOO_FEW_BEATS:       return "SIGNAL_QUALITY_TOO_FEW_BEATS";
        case SIGNAL_QUALITY_INVALID_BEAT_TIMING: return "SIGNAL_QUALITY_INVALID_BEAT_TIMING";
        default: return "UNKNOWN_STATUS";
    }
}

static int join_path(char *dst, size_t cap, const char *dir, const char *file)
{
    int n = snprintf(dst, cap, "%s/%s", dir, file);
    return n >= 0 && (size_t)n < cap;
}

static int load_ppg(const char *path, double *ppg, int *n)
{
    FILE *f = fopen(path, "r");
    char row[MAX_ROW];
    *n = 0;
    if (!f) { perror(path); return 0; }
    if (!fgets(row, sizeof row, f)) { fprintf(stderr, "Empty CSV: %s\n", path); fclose(f); return 0; }

    while (fgets(row, sizeof row, f)) {
        char *first = strchr(row, ',');
        char *second = first ? strchr(first + 1, ',') : NULL;
        char *end;
        double value;
        if (!second) { fprintf(stderr, "Malformed CSV row in %s\n", path); fclose(f); return 0; }
        errno = 0;
        value = strtod(second + 1, &end); /* deliberately accepts NaN and infinity */
        if (end == second + 1 || errno == ERANGE) {
            fprintf(stderr, "Invalid PPG value in %s\n", path); fclose(f); return 0;
        }
        while (*end == ' ' || *end == '\t' || *end == '\r' || *end == '\n') end++;
        if (*end != '\0') { fprintf(stderr, "Unexpected PPG text in %s\n", path); fclose(f); return 0; }
        if (*n >= MAX_TEST_SAMPLES) { fprintf(stderr, "Too many samples in %s\n", path); fclose(f); return 0; }
        ppg[(*n)++] = value;
    }
    if (ferror(f)) { perror(path); fclose(f); return 0; }
    fclose(f);
    return 1;
}

static int load_beats(const char *path, int *beats, int *n)
{
    FILE *f = fopen(path, "r");
    char row[8192];
    char *p, *end;
    *n = 0;
    if (!f) { perror(path); return 0; }
    if (!fgets(row, sizeof row, f)) { fprintf(stderr, "Empty beats: %s\n", path); fclose(f); return 0; }
    fclose(f);
    p = row;
    while (*p) {
        long value;
        while (*p == ' ' || *p == '\t' || *p == ',' || *p == '\r' || *p == '\n') p++;
        if (!*p) break;
        errno = 0;
        value = strtol(p, &end, 10);
        if (p == end || errno || value < INT_MIN || value > INT_MAX || *n >= MAX_TEST_BEATS) {
            fprintf(stderr, "Malformed beats: %s\n", path); return 0;
        }
        beats[(*n)++] = (int)value;
        p = end;
        if (*p && *p != ',' && *p != ' ' && *p != '\t' && *p != '\r' && *p != '\n') {
            fprintf(stderr, "Bad separator in %s\n", path); return 0;
        }
    }
    return 1;
}

/* manifest.csv fields: case, expected_status, num_samples, sample_rate_hz, num_beats, description
   The trailing description is ignored, so commas inside quoted descriptions are harmless. */
int main(int argc, char **argv)
{
    const char *dir = argc > 1 ? argv[1] : "ppg_quality_test_cases";
    char manifest_path[MAX_PATH_LEN], file_path[MAX_PATH_LEN], file_name[256];
    char row[MAX_ROW];
    double ppg[MAX_TEST_SAMPLES];
    int beats[MAX_TEST_BEATS];
    FILE *manifest;
    int passed = 0, failed = 0, errors = 0;

    if (!join_path(manifest_path, sizeof manifest_path, dir, "manifest.csv")) {
        fprintf(stderr, "Directory path too long\n"); return 2;
    }
    manifest = fopen(manifest_path, "r");
    if (!manifest) { perror(manifest_path); return 2; }
    if (!fgets(row, sizeof row, manifest)) { fprintf(stderr, "Empty manifest\n"); fclose(manifest); return 2; }

    printf("Testing SignalQuality.c against: %s\n\n", dir);
    while (fgets(row, sizeof row, manifest)) {
        char case_name[128], expected[128];
        int declared_samples, declared_beats, n_samples, n_beats;
        double sample_rate;
        SignalQualityResult result;
        int ok;

        if (sscanf(row, "%127[^,],%127[^,],%d,%lf,%d",
                   case_name, expected, &declared_samples, &sample_rate, &declared_beats) != 5) {
            fprintf(stderr, "Malformed manifest row: %s", row); errors++; continue;
        }
        snprintf(file_name, sizeof file_name, "%s.csv", case_name);
        if (!join_path(file_path, sizeof file_path, dir, file_name) ||
            !load_ppg(file_path, ppg, &n_samples)) { errors++; continue; }
        snprintf(file_name, sizeof file_name, "%s_beats.txt", case_name);
        if (!join_path(file_path, sizeof file_path, dir, file_name) ||
            !load_beats(file_path, beats, &n_beats)) { errors++; continue; }

        if (n_samples != declared_samples || n_beats != declared_beats) {
            fprintf(stderr, "%s: manifest count mismatch (samples %d/%d, beats %d/%d)\n",
                    case_name, n_samples, declared_samples, n_beats, declared_beats);
            errors++; continue;
        }

        result = CheckSignalQuality(ppg, n_samples, sample_rate, beats, n_beats);
        ok = strcmp(status_name(result.status), expected) == 0;
        printf("[%s] %-28s expected: %-37s actual: %s\n",
               ok ? "PASS" : "FAIL", case_name, expected, status_name(result.status));
        printf("       samples=%d beats=%d range=%.6g valid_intervals=%d invalid_intervals=%d\n",
               n_samples, n_beats, result.signal_range,
               result.valid_intervals, result.invalid_intervals);
        if (ok) passed++; else failed++;
    }
    fclose(manifest);

    /* INVALID_INPUT cannot be expressed as a waveform CSV, so test it directly. */
    {
        double ppg_test[10] = {0};
        int beat_test[5] = {0, 1, 2, 3, 4};
        struct { const char *name; SignalQualityResult result; } direct[] = {
            {"null_ppg",      CheckSignalQuality(NULL, 10, 100.0, beat_test, 5)},
            {"null_beats",    CheckSignalQuality(ppg_test, 10, 100.0, NULL, 5)},
            {"zero_rate",     CheckSignalQuality(ppg_test, 10, 0.0, beat_test, 5)},
            {"negative_count",CheckSignalQuality(ppg_test, 10, 100.0, beat_test, -1)}
        };
        size_t j;
        printf("\nDirect invalid-input tests:\n");
        for (j = 0; j < sizeof direct / sizeof direct[0]; j++) {
            int ok = direct[j].result.status == SIGNAL_QUALITY_INVALID_INPUT;
            printf("[%s] %-28s actual: %s\n", ok ? "PASS" : "FAIL",
                   direct[j].name, status_name(direct[j].result.status));
            if (ok) passed++; else failed++;
        }
    }

    printf("\nSUMMARY: %d passed, %d failed, %d file errors\n", passed, failed, errors);
    return failed || errors ? 1 : 0;
}
