// mechfight/1 protocol loop (port of mechfight.run).
package main

import (
	"bufio"
	"encoding/json"
	"fmt"
	"io"
	"os"
	"runtime/debug"
)

var drawLabel string

// DrawLabel mirrors mf.draw.label: one status line above our mech for this decision.
func DrawLabel(s string) { drawLabel = s }

func send(msg M) error {
	bs, err := json.Marshal(msg)
	if err != nil {
		return err
	}
	bs = append(bs, '\n')
	_, err = os.Stdout.Write(bs)
	return err
}

func readLines(ch chan<- []byte) {
	r := bufio.NewReaderSize(os.Stdin, 1<<20)
	for {
		line, err := r.ReadBytes('\n')
		if len(line) > 0 && len(trimSpace(line)) > 0 {
			ch <- line
		}
		if err != nil {
			if err != io.EOF {
				logf("stdin error", err)
			}
			ch <- nil
			return
		}
	}
}

func trimSpace(b []byte) []byte {
	for len(b) > 0 && (b[len(b)-1] == '\n' || b[len(b)-1] == '\r' || b[len(b)-1] == ' ' || b[len(b)-1] == '\t') {
		b = b[:len(b)-1]
	}
	return b
}

func safeNewBot(hello M) (b *Bot) {
	defer func() {
		if e := recover(); e != nil {
			logf("bot init error", e, string(debug.Stack()))
			b = nil
		}
	}()
	return NewBot(hello)
}

func safeAct(b *Bot, obs M) (act M, ok bool) {
	defer func() {
		if e := recover(); e != nil {
			logf("act() failed at tick", obs["tick"], e, string(debug.Stack()))
			act, ok = nil, false
		}
	}()
	return b.Act(obs), true
}

func main() {
	initSettings()
	ch := make(chan []byte, 1024)
	go readLines(ch)
	var bot *Bot
	last := M{}
	for {
		batch := [][]byte{<-ch}
	drain:
		for {
			select {
			case l := <-ch:
				batch = append(batch, l)
			default:
				break drain
			}
		}
		var latest M
		for _, raw := range batch {
			if raw == nil {
				return
			}
			var msg M
			if json.Unmarshal(raw, &msg) != nil {
				continue
			}
			switch ms(msg, "type") {
			case "hello":
				bot = safeNewBot(msg)
				_ = send(M{"type": "ready", "name": "gocool"})
			case "obs":
				latest = msg
			case "end":
				return
			}
		}
		if latest == nil {
			continue
		}
		action := M{}
		drawLabel = ""
		if bot != nil {
			if a, ok := safeAct(bot, latest); ok {
				action = a
				last = M{}
				for _, k := range []string{"move", "aim", "boost"} {
					if v, has := action[k]; has {
						last[k] = v
					}
				}
			} else {
				action = M{}
				for k, v := range last {
					action[k] = v
				}
			}
		}
		action["tick"] = latest["tick"]
		if drawLabel != "" {
			if _, has := action["debug"]; !has {
				action["debug"] = M{"label": drawLabel}
			}
		}
		if err := send(action); err != nil {
			logf(fmt.Sprintf("unserializable action at tick %v: %v", latest["tick"], err))
			fb := M{"tick": latest["tick"]}
			for k, v := range last {
				fb[k] = v
			}
			_ = send(fb)
		}
	}
}
