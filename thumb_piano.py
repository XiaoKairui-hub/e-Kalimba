# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import numpy as np
import sounddevice as sd
import threading
import mido
from mido import Message
import smbus  # 用于PCF8591 ADC模块
import serial
import traceback

# ========== GPIO 设置 ==========
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# ========= 74HC595 引脚 =========
DATA_PIN = 10    # SER 数据输入
SH_CLK = 9       # SH_CP 移位时钟
ST_CLK = 11      # ST_CP 存储时钟

GPIO.setup(DATA_PIN, GPIO.OUT)
GPIO.setup(SH_CLK, GPIO.OUT)
GPIO.setup(ST_CLK, GPIO.OUT)

# 17 个 LED 的位图（0=灭 1=亮）
led_state = 0

EXTRA_LED = 8
GPIO.setup(EXTRA_LED, GPIO.OUT)

# 琴键引脚 (17 键)
keys = [26,19,21,13,20,6,16,5,12,22,25,27,24,17,23,4,18]
notes = [60,62,64,65,67,69,71,72,74,76,77,79,81,83,84,86,88]  # C4起

for k in keys:
    GPIO.setup(k, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# 模式切换开关 (0=本地发声, 1=MIDI)
mode_pin = 7
GPIO.setup(mode_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# ========== PCF8591 模块 (音量控制) ==========
bus = smbus.SMBus(1)
PCF8591_ADDR = 0x48
def read_volume():
    """读取电位器电压(0~255)并映射为0~1。失败时返回 0.5 作为默认值。"""
    if not bus:
        return 0.5
    try:
        bus.write_byte(PCF8591_ADDR, 0x00)
        value = bus.read_byte(PCF8591_ADDR)
        return max(0.0, min(1.0, value / 255.0))
    except Exception as e:
        print("⚠️ 读取音量失败，使用默认值 0.5:", e)
        return 0.5

# ========== 74HC595 控制函数 ==========
def shift_out(byte2, byte1):
    """依次输出两个字节到两片 74HC595"""
    GPIO.output(ST_CLK, GPIO.LOW)
    for byte in (byte1, byte2):
        for i in range(8):
            bit = (byte >> (7 - i)) & 1
            GPIO.output(DATA_PIN, bit)
            GPIO.output(SH_CLK, GPIO.HIGH)
            GPIO.output(SH_CLK, GPIO.LOW)
    GPIO.output(ST_CLK, GPIO.HIGH)


def update_led():
    global led_state
    byte_low  = led_state & 0xFF
    byte_high = (led_state >> 8) & 0xFF
    shift_out(byte_high, byte_low)

# ========== 声音系统 (保持不变) ==========
sample_rate = 44100
audio_lock = threading.Lock()  # 添加线程锁来保护音频混音操作

def sine_wave(freq, duration, volume):
    """更接近卡林巴：不谐性谐波 + 指数衰减 + 短促敲击噪声"""
    t = np.linspace(0, duration, int(sample_rate * duration), False)
    wave = np.zeros_like(t)

    # 谐波基数和随力度变化的亮度控制
    base_amps = [1.0, 0.6, 0.35, 0.18, 0.08]
    brightness = 0.5 + 0.5 * volume  # 强按更亮
    inharmonicity = 0.0008  # 控制不谐性强弱

    for n, amp in enumerate(base_amps, start=1):
        # 给高次谐波稍微偏移频率以模拟金属/钉子振动的不谐性
        f_n = freq * n * (1.0 + inharmonicity * (n ** 2))
        # 各谐波的指数衰减：高次谐波通常衰减更快
        tau = 0.25 * (1.0 / n + 0.5)  # 调整以获得自然衰减
        env_h = np.exp(-t / tau)
        wave += amp * brightness * np.sin(2 * np.pi * f_n * t) * env_h

    # 添加短促的击弦瞬态噪声（滤波白噪）
    noise = np.random.randn(len(t)) * 0.8
    # 指数衰减的瞬态，持续约 8-20 ms
    transient_tau = 0.012
    transient_env = np.exp(-t / transient_tau)
    # 简单一阶低通滤波（平滑噪声高频）
    alpha = 0.04
    filtered = np.zeros_like(noise)
    filtered[0] = noise[0] * alpha
    for i in range(1, len(noise)):
        filtered[i] = filtered[i-1] + alpha * (noise[i] - filtered[i-1])
    wave += 0.15 * filtered * transient_env  # 瞬态比例可调

    # 总体包络（当做轻微的ADSR参考 + 保证尾部平滑）
    adsr = adsr_envelope(duration, attack=0.008, decay=0.06, sustain=0.7, release=0.12)
    wave *= adsr

    # 归一化避免削波（按最大绝对值缩放）
    maxv = np.max(np.abs(wave))
    if maxv > 0:
        wave = wave / maxv * (0.8 * volume)

    return wave.astype(np.float32)

def adsr_envelope(duration, attack=0.02, decay=0.1, sustain=0.6, release=0.15):
    """组合线性 ADSR 和末尾平滑，输出 0..1 的包络"""
    samples = int(sample_rate * duration)
    env = np.zeros(samples)
    attack_s = max(1, int(sample_rate * attack))
    decay_s = max(1, int(sample_rate * decay))
    release_s = max(1, int(sample_rate * release))
    sustain_s = max(0, samples - (attack_s + decay_s + release_s))

    # attack
    env[:attack_s] = np.linspace(0.0, 1.0, attack_s, endpoint=False)
    # decay
    env[attack_s:attack_s+decay_s] = np.linspace(1.0, sustain, decay_s, endpoint=False)
    # sustain
    start = attack_s + decay_s
    env[start:start+sustain_s] = sustain
    # release
    if release_s > 0:
        env[-release_s:] = np.linspace(sustain, 0.0, release_s)

    # 在主体上再乘以一个轻微的指数衰减，让尾部更像金属类振动
    t = np.linspace(0, duration, samples, False)
    env *= np.exp(-t * 1.2 / duration)

    return env

# 创建一个全局的音频混音器
class AudioMixer:
    def __init__(self):
        # current_waves: note -> dict{ 'wave': np.array, 'position': int, 'releasing': bool,
        #                              'release_len': int, 'release_pos': int }
        self.current_waves = {}
        # 增大 blocksize / latency 减少 underrun 风险；可根据效果调大或调小
        try:
            self.stream = sd.OutputStream(
                channels=1,
                samplerate=sample_rate,
                dtype='float32',
                blocksize=1024,   # 或 2048，如果仍有 underrun 可增大
                latency=0.08,     # 秒级延迟允许（0.02-0.2 之间试验）
                callback=self._audio_callback
            )
            self.stream.start()
        except Exception as e:
            print("⚠️ 打开音频流失败：", e)
            raise

    def _audio_callback(self, outdata, frames, time, status):
        with audio_lock:
            if self.current_waves:
                mix = np.zeros(frames, dtype=np.float32)
                to_remove = []
                for note_id, info in list(self.current_waves.items()):
                    wave = info['wave']
                    pos = info.get('position', 0)
                    releasing = info.get('releasing', False)
                    release_len = info.get('release_len', 0)
                    release_pos = info.get('release_pos', 0)

                    remaining = len(wave) - pos

                    if not releasing:
                        if remaining <= 0:
                            to_remove.append(note_id)
                            continue
                        n_samples = min(frames, remaining)
                        mix[:n_samples] += wave[pos:pos + n_samples]
                        info['position'] = pos + n_samples
                    else:
                        # 构建一帧长度的缓冲块，前部用原始波形（若还有），其余为0
                        chunk = np.zeros(frames, dtype=np.float32)
                        n_wave = min(frames, max(0, remaining))
                        if n_wave > 0:
                            chunk[:n_wave] = wave[pos:pos + n_wave]

                        # 计算逐样本的释放包络（线性淡出），超出 release_len 的部分为 0
                        if release_len > 0:
                            env = 1.0 - (np.arange(frames) + release_pos) / float(release_len)
                            env = np.clip(env, 0.0, 1.0)
                            chunk *= env
                        else:
                            chunk *= 0.0

                        mix += chunk
                        info['position'] = pos + n_wave
                        info['release_pos'] = release_pos + frames

                        # 当释放包络完成或波形已耗尽且包络为0时移除
                        if info['release_pos'] >= release_len:
                            to_remove.append(note_id)

                for nid in to_remove:
                    try:
                        del self.current_waves[nid]
                    except KeyError:
                        pass

                # 简单幅度控制：按最大同时声部数归一化以避免削波
                active = len(self.current_waves) if len(self.current_waves) > 0 else 1
                mix = mix / max(1.0, active ** 0.9)

                outdata[:, 0] = mix
            else:
                outdata.fill(0)

    def add_wave(self, note, wave):
        """立即开始播放（按下时调用）"""
        with audio_lock:
            self.current_waves[note] = {
                'wave': wave,
                'position': 0,
                'releasing': False,
                'release_len': 0,
                'release_pos': 0
            }

    def release_wave(self, note, release_time=0.6):
        """标记为释放（按键松开时调用），release_time 单位秒"""
        with audio_lock:
            if note in self.current_waves:
                info = self.current_waves[note]
                pos = info.get('position', 0)
                remaining = max(0, len(info['wave']) - pos)
                # release_len 不应超过剩余波形长度（否则会无效的零填充）
                desired = int(sample_rate * release_time)
                info['releasing'] = True
                info['release_pos'] = 0
                info['release_len'] = min(max(1, desired), max(1, remaining))

    def remove_wave_immediate(self, note):
        """立即移除（保留用于必要时强停）"""
        with audio_lock:
            if note in self.current_waves:
                del self.current_waves[note]

# 创建全局混音器实例
mixer = AudioMixer()

def play_note_thread(note, volume):
    freq = 440 * 2 ** ((note - 69) / 12)
    # 延长时长以便自然衰减（拇指琴通常有较长的尾音）
    wave = sine_wave(freq, 1.2, volume)
    mixer.add_wave(note, wave)

def play_local_sound(note, volume):
    t = threading.Thread(target=play_note_thread, args=(note, volume), daemon=True)
    t.start()

def stop_local_sound(note):
    # 改为触发释放（延音），而不是立即删除
    # release_time 可根据需要调整（单位秒）
    mixer.release_wave(note, release_time=0.6)

# ========== MIDI 输出 (修改) ==========
outport = None
if mido:
    try:
        # 本地虚拟端口仍然保留，用于在 RPi 上调试
        outport = mido.open_output('Kalimba_Virtual', virtual=True)
        print("✅ 已创建本地虚拟MIDI端口 'Kalimba_Virtual' (用于调试)")
    except Exception as e:
        outport = None
        print("⚠️ 无法创建虚拟MIDI端口:", e)
else:
    print("⚠️ mido 未安装或不可用。")

# ========== [新增] RPi -> ESP32 串口 (UART) 设置 ==========
ser = None
try:
    # /dev/serial0 是 RPi GPIO 串口的稳定别名
    # 波特率 115200 (必须与你的 ESP32 代码设置一致)
    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    print(f"✅ 成功打开 ESP32 串口: {ser.name} (波特率 115200)")
except Exception as e:
    print(f"CRITICAL ⚠️: 无法打开串口 /dev/serial0: {e}")
    print("     请检查：")
    print("     1. ESP32 是否已连接到 RPi 的 TX/RX 引脚？ (RPi TX -> ESP32 RX, RPi RX -> ESP32 TX)")
    print("     2. 是否已在 /boot/config.txt 中启用了 UART？ (添加 'dtoverlay=miniuart-bt' 和 'enable_uart=1')")
    print("     3. 是否已通过 sudo raspi-config 禁用了 'Serial Console'？")
    print("     MIDI 模式将无法发送到 ESP32。")
    ser = None

# ========== [修改] 发送 MIDI (同时发到虚拟端口和 ESP32) ==========

def send_midi_note_on(note, velocity):
    # 使用 mido 构造标准 MIDI 消息
    msg = Message('note_on', note=note, velocity=int(velocity))
    
    # 1. (可选) 发送到本地虚拟端口，用于 RPi 上的调试
    if outport is not None:
        try:
            outport.send(msg)
        except Exception as e:
            print(f"⚠️ 发送 note_on 到虚拟端口失败: {e}")
    
    # 2. [核心] 将原始 MIDI 字节发送到 ESP32
    if ser is not None:
        try:
            # msg.bytes() 会返回一个字节列表，例如 [0x90, 60, 127]
            ser.write(msg.bytes())
        except Exception as e:
            print(f"⚠️ 发送 note_on 到 ESP32 失败: {e}")

def send_midi_note_off(note):
    msg = Message('note_off', note=note, velocity=0)
    
    # 1. (可选) 发送到本地虚拟端口
    if outport is not None:
        try:
            outport.send(msg)
        except Exception as e:
            print(f"⚠️ 发送 note_off 到虚拟端口失败: {e}")
            
    # 2. [核心] 将原始 MIDI 字节发送到 ESP32
    if ser is not None:
        try:
            # msg.bytes() 会返回一个字节列表，例如 [0x80, 60, 0]
            ser.write(msg.bytes())
        except Exception as e:
            print(f"⚠️ 发送 note_off 到 ESP32 失败: {e}")

# ========== 主循环 (保持不变) ==========
print("🎹 Dual-mode Kalimba (RPi + ESP32)")
print(f"GPIO{mode_pin}=LOW 本地发声, GPIO{mode_pin}=HIGH MIDI 输出到 ESP32")

prev_state = [GPIO.input(k) for k in keys]
playing_notes = {}

# 在主循环前添加去抖相关变量
DEBOUNCE_TIME = 0.05  # 50ms 去抖时间
last_press_time = {k: 0 for k in keys}  # 记录每个按键最后一次按下的时间

try:
    while True:
        mode = GPIO.input(mode_pin)  # 当前模式
        volume = read_volume()       # 实时读取音量
        velocity = int(volume * 127) # 转换为MIDI力度(0~127)
        current_state = [GPIO.input(k) for k in keys]
        current_time = time.time()   # 获取当前时间

        # 处理最后一个键（单独的LED）
        if prev_state[16] == GPIO.HIGH and current_state[16] == GPIO.LOW:
            if (current_time - last_press_time[keys[16]]) > DEBOUNCE_TIME:
                note = notes[16]
                GPIO.output(EXTRA_LED, GPIO.HIGH)
                if mode == GPIO.LOW or mode == 0:
                    play_local_sound(note, volume)
                else:
                    send_midi_note_on(note, velocity)
                playing_notes[keys[16]] = note
                print(f"按下键 {16+1}: Note {note}, 音量 {volume:.2f}")
                last_press_time[keys[16]] = current_time

        elif prev_state[16] == GPIO.LOW and current_state[16] == GPIO.HIGH:
            if (current_time - last_press_time[keys[16]]) > DEBOUNCE_TIME:
                GPIO.output(EXTRA_LED, GPIO.LOW)
                if keys[16] in playing_notes:
                    note = playing_notes.pop(keys[16])
                    if mode != GPIO.LOW and mode != 0:
                        send_midi_note_off(note)
                    print(f"松开键 {16+1}: Note {note}")
                last_press_time[keys[16]] = current_time

        # 处理其他键
        for i in range(len(keys)-1):
            if prev_state[i] == GPIO.HIGH and current_state[i] == GPIO.LOW:
                if (current_time - last_press_time[keys[i]]) > DEBOUNCE_TIME:
                    note = notes[i]
                    led_state |= (1 << i)  # 第 i 位置 1
                    update_led()
                    if mode == GPIO.LOW or mode == 0:
                        play_local_sound(note, volume)
                    else:
                        send_midi_note_on(note, velocity)
                    playing_notes[keys[i]] = note
                    print(f"按下键 {i+1}: Note {note}, 音量 {volume:.2f}")
                    last_press_time[keys[i]] = current_time

            elif prev_state[i] == GPIO.LOW and current_state[i] == GPIO.HIGH:
                if (current_time - last_press_time[keys[i]]) > DEBOUNCE_TIME:
                    led_state &= ~(1 << i)  # 第 i 位清 0
                    update_led()
                    if keys[i] in playing_notes:
                        note = playing_notes.pop(keys[i])
                        if mode == 0:  # 本地音频模式
                            stop_local_sound(note)  # 添加这一行
                        elif mode == 1:  # MIDI模式
                            send_midi_note_off(note)
                        print(f"松开键 {i+1}: Note {note}")
                    last_press_time[keys[i]] = current_time

        prev_state = current_state
        time.sleep(0.01)

except KeyboardInterrupt:
    print("🎵 程序收到中断，正在退出...")
    mixer.stream.stop()
    mixer.stream.close()
    GPIO.cleanup()
    print("🎵 程序结束，资源已释放。")

finally:
    try:
        sd.stop()
    except Exception:
        pass
    if outport is not None:
        try:
            outport.close()
        except Exception:
            pass
            
    # [新增] 关闭串口
    if ser is not None and ser.is_open:
        try:
            ser.close()
            print("✅ 串口已关闭。")
        except Exception as e:
            print(f"⚠️ 关闭串口失败: {e}")
            
    try:
        GPIO.cleanup()
    except Exception:
        pass
    print("🎵 程序结束，资源已释放。")