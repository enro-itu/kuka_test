# README2.md

## Proje Derleme ve Çalıştırma Adımları

Bu proje ROS2 ortamında derlenip çalıştırılmak üzere hazırlanmıştır. Aşağıdaki adımlar, terminal üzerinden çalıştırılması gereken komutları içermektedir.

---

### 1. Çalışma Alanını Oluşturma

Öncelikle ROS2 çalışma alanını (workspace) oluşturun:

```bash
mkdir rk_ws
cd ~/rk_ws
```

---

### 2. Derleme İşlemi

Projeyi aşağıdaki komutla derleyin:

```bash
colcon build --packages-select rk_demo --symlink-install
```

> **Not:** `--symlink-install` parametresi, kaynak dosyalarda yapılan değişikliklerin doğrudan yansıtılmasını sağlar.

---

### 3. Ortam Değişkenlerini Güncelleme

Derleme tamamlandıktan sonra, yeni oluşturulan `install/setup.bash` dosyasını kaynak (source) edinerek ROS2 ortam değişkenlerini güncelleyin:

```bash
source ~/rk_ws/install/setup.bash
```

---

### 4. Projeyi Çalıştırma

Proje başarıyla derlendikten sonra aşağıdaki komutla çalıştırabilirsiniz:

```bash
ros2 launch rk_demo display.launch.py
```

> **Not:** Gazebo entegrasyonu şu anda tam olarak çalışmamaktadır. Görselleştirme veya simülasyon ortamında hata alırsanız, yalnızca `display.launch.py` dosyası üzerinden devam etmeniz önerilir.

---

### 5. Ek Bilgiler

- Çalışma alanı: `~/rk_ws`
- Paket adı: `rk_demo`
- ROS2 sürümü: (örnek) Humble / Jazzy
- Derleme aracı: `colcon`

---

📘 **Hazırlayan:** Yağız Çümen  
💡 **Açıklama:** Bu belge, ROS2 ortamında `rk_demo` paketinin hızlı şekilde derlenmesi ve çalıştırılması için hazırlanmıştır.

