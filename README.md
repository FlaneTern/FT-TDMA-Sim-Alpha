# FT-TMDA-Sim (Fault-Tolerant Time-Division Multiple Access Simulator)

FT-TDMA-Sim adalah program komputer untuk menyimulasikan kerja WSN. FT-TDMA-Sim dirancang untuk memudahkan pengguna memodifikasi programnya dan mengatur metode untuk menentukan data transfer interval, routing, dan cara kerja simulator.


## Setup
FT-TDMA-Sim memiliki support untuk Windows dan Linux. Untuk meng-setup FT-TDMA-Sim,
pertama, clone source code dari repository GitHub dengan menggunakan perintah :
```sh
git clone https://github.com/FlaneTern/FT-TDMA-Sim.git
```

### Setup pada Windows
- Eksekusi program ”BUILD WINDOWS.bat” untuk men-generate solution dan project files Visual Studio 2022.
- Buka solution file ”FT-TDMA-Sim.sln” untuk membuka solution dengan Visual Studio.


### Setup pada Linux
- Eksekusi program "BUILD LINUX.sh" untuk men-generate makefile dengan perintah :
```sh
./BUILD_LINUX.sh
```
- Gunakan code editor favorit pada Linux untuk membuka folder FT-TDMA-Sim.




## "Problem" class
Sebuah problem terdiri dari daftar lokasi SN dan timestamp kegagalan untuk setiap SN. Secara default,
lokasi untuk 100 SN akan di-generate dengan distribusi uniform U ∈ [−1000, 1000]. Kemudian, timestamp kegagalan untuk setiap SN di-generate dengan distribusi eksponensial dengan μ = σ = 3600×24.
Sebuah problem dapat ”memiliki” beberapa simulator. Ketika method Run() pada problem dipanggil, maka akan dijalankan simulasi menggunakan seluruh simulator yang dimiliki oleh problem.

## "Simulator" class
Sebuah simulator dapat menyimulasikan kerja WSN pada sebuah problem case. Simulator memiliki
berbagai parameter yang terangkum dalam struct ”SimulatorParameter”, yang terdiri dari :
1. TotalDurationToBeTransferred : Durasi total untuk menjalankan simulasi
2. TransferTime : Durasi yang diperlukan sebuah SN untuk melakukan data transfer.
3. RecoveryTime : Durasi yang diperlukan sebuah SN untuk perbaikan
4. EnergyRateWorking : Jumlah energi yang digunakan sebuah SN per satuan waktu untuk
melakukan data sensing.
5. EnergyRateTransfer : Jumlah energi yang digunakan sebuah SN per satuan waktu untuk
melakukan data transfer.
6. TransmissionRange : Jangkauan transmisi data sebuah SN.
7. InterferenceRange : Jangkauan interferensi sebuah SN.
Ketika sebuah simulator diinisialisasi, simulator tersebut akan mendapatkan ID dan menyimpan
parameter-parameter yang telah diberikan. Kemudian, ketika simulator dijalankan dengan memang-
gil method Run() (yang akan dipanggil oleh problem), simulator akan melakukan beberapa langkah
berikut :
1. Konstruksi Topologi : Topologi dari WSN dikonstruksi dengan cara :

    (a) Mencari seluruh SN yang berada dalam jarak TransmissionRange dari BS dan menghubungkan-
nya ke BS.

    (b) Untuk seluruh SN yang belum masuk di topologi, cari SN terdekat dalam jangkauan Trans-
missionRange yang telah masuk di topologi dan merupakan leaf pada iterasi sebelumnya.
Kemudian, hubungkan SN yang belum masuk di topologi dengan SN yang telah memenuhi
syarat tersebut.

    (c) Ulangi langkah (b) hingga seluruh SN sudah masuk di topologi atau tidak ada SN yang
masih bisa masuk ke topologi.
2. Pewarnaan Graph : Pewarnaan graph topologi penting untuk menentukan kelompok dari
setiap SN dalam penjadwalan TDMA. Pewarnaan topologi ini dilakukan dengan menggunakan
algoritma Welsh-Powell dengan mempertimbangkan lokasi SN dan interference range. Secara
singkat, langkah-langkah algoritma graph coloring ini adalah sebagai berikut :

    (a) Cari degree dari setiap SN. Degree dari SN adalah jumlah SN lain yang berada dalam
interference range dari SN tersebut.

    (b) Urutkan SN secara descending berdasarkan degree.

    (c) Warnai SN pertama dengan warna 1.
    
    (d) Iterasi pada array SN dan warnai semua SN yang tidak berada dalam interference range
dari seluruh SN lainnya dengan warna yang sama.

    (e) Ulangi langkah (d) untuk semua SN yang belum diwarnai dengan menggunakan warna baru,
hingga semua SN memiliki warna.
3. Pengaturan ∆ : Milai ∆ diatur untuk setiap SN. Secara default, nilai ini diacak dengan
distribusi uniform U ∈ [1000 − √3, 1000 + √3]
4. Simulasi : Simulator menyimulasikan kerja WSN berdasarkan parameter, topologi, pewarnaan,
dan nilai ∆ yang telah diatur sebelumnya. Cara kerja default simulasi ini dapat dilihat pada
lampiran source code ”Simulator.cpp”. Simulator menginisialisasi sebuah priority queue yang
menyimpan data mengenai transisi state selanjutnya untuk setiap SN, meliputi timestamp dan
jenis state. Transisi yang memiliki timestamp lebih kecil akan diproses terlebih dahulu.
Seluruh SN diinisialisasi dengan state Data Sensing pada timestamp 0. Seluruh transisi ini
dimasukkan ke priority queue. Kemudian, akan dilakukan perulangan yang menyimulasikan
perubahan keadaan WSN pada setiap transisi state SN. Perulangan ini dilakukan selama kondisi
terminasi belum tercapai. Pada perulangan ini, dilakukan beberapa hal berikut :

    (a) Penentuan state selanjutnya : Apabila tidak terjadi kegagalan pada SN, state selanjut-
nya dari SN tersebut ditentukan oleh state sekarang SN tersebut, dengan aturan transisi:

    • Collection → Transfer

    • Transfer → Collection

    • Recovery → Collection

Namun, apabila terjadi kegagalan SN sebelum waktu singgah pada state yang sekarang
selesai, maka state selanjutnya dari SN tersebut adalah Recovery. Timestamp transisi selanjutnya ditentukan berdasarkan timestamp transisi sekarang dan durasi singgah pada
masing-masing state, yaitu TransferTime untuk state Transfer dan RecoveryTime untuk
state Recovery. Khusus pada transisi Collection → Transfer, timestamp transisi selanjutnya adalah timeslot untuk data transfer kelompok warna SN tersebut yang paling dekat
dengan timestamp sekarang ditambah dengan ∆ SN. Transisi selanjutnya tersebut kemu-
dian dimasukkan ke priority queue.
    
(b) Menangani Transisi State :

    i. Collection → Transfer : The node’s data collection time, energy consumption, and data packet size are updated.

    ii. Transfer → Collection : If the node’s parent is not the base station and isn’t in recovery,
    data is forwarded to the parent node. Otherwise, the data is considered successfully
    transferred to the base station. Energy consumption and wasted time are calculated.

    iii. Recovery → Collection : Resets the node’s packets, indicating that the recovery period
    is over.

(c) Cek Kondisi Terminasi : Dilakukan pengecekan untuk syarat terminasi. Secara default,
simulasi akan dihentikan apabila waktu pada simulasi sudah sama dengan atau lebih dari
TotalDurationToBeTransferred.

## Main Function
Cara kerja fungsi main adalah sebagai berikut :
1. Inisialisasi problem : Inisialisasikan sebuah (atau beberapa) problem instance dengan me-
manggil constructor class Problem.
2. Tentukan parameter simulator : Inisialisasikan SimulatorParameter dengan memberikan
nilai yang diinginkan untuk setiap parameter. Apabila hendak dilakukan simulasi dengan berba-
gai nilai untuk setiap parameter, dapat dikonstruksikan sebuah SimulatorParameterGrid, yang
mengandung array dari nilai-nilai berbeda untuk masing-masing parameter.
3. Inisialisasi simulator : Inisialisasikan simulator atau array dari simulator dengan memanggil
Simulator::CreateSimulator() dan memberikan argumen berupa SimulatorParameter atau Simu-
latorParameterGrid yang telah dikonstruksi sebelumnya.
4. Tambahkan simulator ke problem : Tambahkan simulator-simulator yang diinginkan ke
dalam problem dengan memanggil method AddSimulator() pada problem dan memberikan simulator sebagai argumennya.
5. Panggil method Run() : Panggil method Run() pada problem untuk menjalankan seluruh
simulator yang telah ditambahkan ke problem tersebut.

## Build and Run

### Build and Run pada Windows
Pilih build configuration yang diinginkan dan tekan Start Debugging

### Build and Run pada Linux
Untuk melakukan kompilasi, jalankan salah satu dari kedua perintah berikut sesuai dengan konfigurasi
yang diinginkan :
```sh
    make config=debug_x64
    make config=release_x64
```

Kemudian, untuk menjalankan program yang telah terkompilasi, jalankan salah satu dari kedua perintah berikut sesuai dengan konfigurasinya :

```sh
    ./bin/Debug-x86_64/FT-TDMA-Sim/FT-TDMA-Sim
    ./bin/Release-x86_64/FT-TDMA-Sim/FT-TDMA-Sim
```
