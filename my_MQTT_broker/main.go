package main

import (
	"bufio"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"strings"
	"syscall"

	mqtt "github.com/mochi-mqtt/server/v2"
	"github.com/mochi-mqtt/server/v2/hooks/auth"
	"github.com/mochi-mqtt/server/v2/listeners"
	"github.com/mochi-mqtt/server/v2/packets"
)

func main() {
	// 1. 建立 MQTT Broker（必須設置 InlineClient: true）
	server := mqtt.New(&mqtt.Options{
		InlineClient: true, // 啟用內部直接發布/訂閱功能
	})

	// 2. 允許匿名連線
	_ = server.AddHook(new(auth.AllowHook), nil)

	// 3. 建立 TCP Listener (Port 1883)
	tcp := listeners.NewTCP(listeners.Config{
		ID:      "t1",
		Address: ":1883",
	})
	if err := server.AddListener(tcp); err != nil {
		log.Fatalf("TCP Listener 建立失敗: %v", err)
	}

	// 4. 啟動 Broker 服務
	go func() {
		if err := server.Serve(); err != nil {
			log.Fatalf("Broker 運作異常: %v", err)
		}
	}()

	fmt.Println("==================================================")
	fmt.Println("🚀 Golang MQTT Broker 已於 Port 1883 啟動！")
	fmt.Println("==================================================")

	// 5. 內聯訂閱 (Inline Subscribe)：即時接收來自 ESP32 的遙測數據
	err := server.Subscribe("ts_rmu/telemetry", 1, func(cl *mqtt.Client, sub packets.Subscription, pk packets.Packet) {
		fmt.Printf("\r[📡 收到 ESP32 數據] %s\n請輸入指令 > ", string(pk.Payload))
	})
	if err != nil {
		log.Printf("訂閱遙測主題失敗: %v", err)
	}

	// 6. 開放 HTTP REST API (Port 8080)，支援網頁或 Curl 控制
	go func() {
		http.HandleFunc("/api/cmd", func(w http.ResponseWriter, r *http.Request) {
			action := r.URL.Query().Get("action") // e.g. /api/cmd?action=START_CYCLE
			if action == "" {
				http.Error(w, "缺少 'action' 參數", http.StatusBadRequest)
				return
			}
			cmd := strings.ToUpper(action)
			
			// 向 ESP32 的控制 Topic 發送指令
			err := server.Publish("ts_rmu/cmd", []byte(cmd), false, 0)
			if err != nil {
				http.Error(w, fmt.Sprintf("發送失敗: %v", err), http.StatusInternalServerError)
				return
			}
			fmt.Fprintf(w, "OK! 已傳送指令: %s 至 ts_rmu/cmd", cmd)
			fmt.Printf("\n[🌐 HTTP 下達指令] %s -> ts_rmu/cmd\n請輸入指令 > ", cmd)
		})

		log.Println("🌐 HTTP 控制介面就緒: http://localhost:8080/api/cmd?action=START_CYCLE")
		_ = http.ListenAndServe(":8080", nil)
	}()

	// 7. 終端互動介面 (CLI Console)
	go func() {
		scanner := bufio.NewScanner(os.Stdin)
		printHelp()

		for scanner.Scan() {
			input := strings.TrimSpace(scanner.Text())
			switch input {
			case "1":
				_ = server.Publish("ts_rmu/cmd", []byte("START_CYCLE"), false, 0)
				fmt.Println("[📤 已發送指令] START_CYCLE -> 啟動 4s On / 1s Off 循環採集")
			case "2":
				_ = server.Publish("ts_rmu/cmd", []byte("STOP_CYCLE"), false, 0)
				fmt.Println("[📤 已發送指令] STOP_CYCLE -> 停止採集，恢復常態閉合")
			case "3":
				_ = server.Publish("ts_rmu/cmd", []byte("SINGLE_TEST"), false, 0)
				fmt.Println("[📤 已發送指令] SINGLE_TEST -> 觸發單次 300ms 斷電測試")
			case "h", "help":
				printHelp()
			default:
				if input != "" {
					_ = server.Publish("ts_rmu/cmd", []byte(input), false, 0)
					fmt.Printf("[📤 已發送自訂指令] %s -> ts_rmu/cmd\n", input)
				}
			}
			fmt.Print("請輸入指令 > ")
		}
		if err := scanner.Err(); err != nil {
			log.Printf("讀取終端輸入失敗: %v", err)
		}
	}()

	// 8. 優雅關閉 (Graceful Shutdown)
	done := make(chan os.Signal, 1)
	signal.Notify(done, syscall.SIGINT, syscall.SIGTERM)
	<-done

	fmt.Println("\n正在關閉 Broker...")
	_ = server.Close()
}

func printHelp() {
	fmt.Println("\n---------------- 可用控制指令 ----------------")
	fmt.Println("  [1] 輸入 1 -> 發送 'START_CYCLE' (啟動 4s ON / 1s OFF)")
	fmt.Println("  [2] 輸入 2 -> 發送 'STOP_CYCLE'  (停止採集)")
	fmt.Println("  [3] 輸入 3 -> 發送 'SINGLE_TEST' (單次 300ms 斷電)")
	fmt.Println("  [h] 輸入 h -> 顯示選單說明")
	fmt.Println("--------------------------------------------------")
}