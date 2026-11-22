# Gocool mech bot (Go 1:1 port of bot.py). Build: docker build -t gocool .
FROM golang:1.22 AS build
WORKDIR /src
COPY go.mod ./
COPY cmd ./cmd
RUN CGO_ENABLED=0 GOOS=linux GOARCH=amd64 go build -trimpath -ldflags="-s -w" -o /out/bot ./cmd/bot

FROM scratch
WORKDIR /bot
COPY --from=build /out/bot /bot/bot
COPY navmap.npz /bot/navmap.npz
USER 65534:65534
ENTRYPOINT ["/bot/bot"]
