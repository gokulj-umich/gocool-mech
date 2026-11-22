# Gocool mech bot, Go port (v70). Build: docker build -t gocool .
FROM golang:1.22-bookworm AS build
WORKDIR /src
COPY go.mod ./
COPY cmd ./cmd
ENV CGO_ENABLED=0 GOOS=linux GOARCH=amd64
RUN go build -trimpath -ldflags="-s -w" -o /out/mechbot ./cmd/bot

FROM scratch
COPY --from=build /out/mechbot /mechbot
USER 65534:65534
ENTRYPOINT ["/mechbot"]
