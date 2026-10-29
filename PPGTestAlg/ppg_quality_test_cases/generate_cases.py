import csv, math, random, json
from pathlib import Path

OUT=Path(__file__).parent
FS=100
N=1000
beats=list(range(50, 1000, 80))
rng=random.Random(26029)

def waveform(n=N):
    values=[]
    for i in range(n):
        # Positive photoplethysmogram-like pulses with a shoulder/reflection.
        t=i/FS
        phase=((i-50)%80)/100.0
        # Gaussian pulse near each labeled beat; values in MAX30102 raw ADC units.
        dist=min(abs(i-b) for b in range(-30, n+120, 80))
        # A second weaker wave occurs 17 samples after each main peak.
        reflection=min(abs(i-(b+17)) for b in range(-30, n+120,80))
        val=52000+6200*math.exp(-0.5*(dist/6.0)**2)+1400*math.exp(-0.5*(reflection/5.0)**2)
        val+=150*math.sin(2*math.pi*0.17*t)+rng.gauss(0,60)
        values.append(round(val,3))
    return values
base=waveform()

cases=[]
def add(name,status,data,beat_list,description,rate=FS):
    # Every CSV includes all input samples. Beat indices are in separate text file, 0-based.
    with (OUT/(name+'.csv')).open('w',newline='') as f:
        w=csv.writer(f)
        w.writerow(['sample_index','timestamp_ms','ppg_raw'])
        for i,v in enumerate(data):
            w.writerow([i,round(i*1000/rate,4),v])
    (OUT/(name+'_beats.txt')).write_text(','.join(map(str,beat_list))+'\n')
    cases.append(dict(case=name,expected_status=status,num_samples=len(data),sample_rate_hz=rate,num_beats=len(beat_list),description=description))

add('01_good','SIGNAL_QUALITY_GOOD',base,beats,'Clean, plausible ~75 BPM synthetic raw PPG')
add('02_flat','SIGNAL_QUALITY_FLAT_SIGNAL',[52000.0]*N,beats,'Constant ADC reading; no pulse')
add('03_very_low_amplitude','SIGNAL_QUALITY_GOOD',[52000+0.00000002*math.sin(2*math.pi*i/80) for i in range(N)],beats,'Near-flat signal that still passes epsilon=1e-9; demonstrates lack of meaningful amplitude threshold')
clipped=base.copy()
for i in range(15,45): clipped[i]=262143.0
add('04_high_rail_clipping','SIGNAL_QUALITY_SATURATED',clipped,beats,'30 of 1000 values equal full-scale MAX30102 ADC code')
lowclip=base.copy()
for i in range(150,180): lowclip[i]=0.0
add('05_low_rail_clipping','SIGNAL_QUALITY_SATURATED',lowclip,beats,'30 of 1000 samples near lower rail')
add('06_too_few_samples','SIGNAL_QUALITY_TOO_FEW_SAMPLES',base[:8],[2,4],'Only 8 samples; short-window check takes precedence')
add('07_nan','SIGNAL_QUALITY_NONFINITE_DATA',base[:350]+['nan']+base[351:],beats,'One NaN sample; C strtod("nan") or NAN needed to load')
add('08_infinity','SIGNAL_QUALITY_NONFINITE_DATA',base[:610]+['inf']+base[611:],beats,'One +infinity sample')
add('09_too_few_beats','SIGNAL_QUALITY_TOO_FEW_BEATS',base,beats[:4],'Only 4 beat indices, despite plausible PPG')
add('10_too_fast_beat','SIGNAL_QUALITY_INVALID_BEAT_TIMING',base,[50,130,210,290,370,390,450,530,610,690,770,850,930],'20-sample interval = 300 BPM, beyond 200 BPM upper threshold')
add('11_too_slow_beat','SIGNAL_QUALITY_INVALID_BEAT_TIMING',base,[50,130,210,290,370,450,530,710,790,870,950],'180-sample interval = 33.3 BPM, below 40 BPM lower threshold')
add('12_duplicate_beat','SIGNAL_QUALITY_INVALID_BEAT_TIMING',base,[50,130,210,290,370,370,450,530,610,690,770,850,930],'Duplicate 370 beat index (0 interval)')
add('13_out_of_range_beat','SIGNAL_QUALITY_INVALID_BEAT_TIMING',base,beats+[1000],'Beat points beyond final array index 999')
add('14_reverse_order_beat','SIGNAL_QUALITY_INVALID_BEAT_TIMING',base,[50,130,210,290,370,450,530,610,690,770,850,930,900],'Last beat is earlier than previous one')
add('15_random_noise','SIGNAL_QUALITY_GOOD',[round(52000+rng.gauss(0,5500),3) for _ in range(N)],beats,'Random electrical/motion noise can PASS if supplied beat indices are plausible: known limitation')
add('16_extremely_high_steady_adc','SIGNAL_QUALITY_GOOD',[round(261000+500*math.sin(2*math.pi*i/80),3) for i in range(N)],beats,'High baseline but not within 2 ADC counts of a rail; passes')

with (OUT/'manifest.csv').open('w',newline='') as f:
    w=csv.DictWriter(f,fieldnames=cases[0].keys());w.writeheader();w.writerows(cases)
(OUT/'README.txt').write_text('''PPG SignalQuality synthetic test set\n\nEach waveform CSV columns: sample_index, timestamp_ms, ppg_raw\nThe matching *_beats.txt contains comma-separated, 0-based beat_indices.\nUse sample_rate=100.0 Hz for all cases, unless manifest says otherwise.\nExpected statuses reflect the EXACT posted CheckSignalQuality() function and its check order.\n\nTo test INVALID_INPUT (requires function arguments, not CSV), call:\n  CheckSignalQuality(NULL, 1000, 100.0, beats, nbeats);\n  CheckSignalQuality(ppg, 1000, 0.0, beats, nbeats);\n  CheckSignalQuality(ppg, 1000, 100.0, NULL, nbeats);\n\nC parsing: use strtod() or equivalent for ppg_raw; it accepts "nan" and "inf".\nDo not parse nonfinite cells as empty/zero.\n\nIMPORTANT: beat indices in this pack are intentionally PROVIDED, not the output of a\nheartbeat detector. To unit-test CheckSignalQuality, pass them directly. If you\nre-run FindHeartbeats() on these waveforms, results and expected status may differ.\nA noisy signal or absurdly small amplitude can pass this quality implementation\nwhen supplied beat indices are valid. No signal-to-noise or pulse morphology\ncheck exists yet. This is intentional for testing.\nThese are artificial waveforms, NOT validated physiological data and NOT adequate\nto qualify a BP estimation device.\n''')
print(len(cases),'cases:',[(c['case'],c['expected_status']) for c in cases])
