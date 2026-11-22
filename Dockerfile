FROM golang:1.22 AS build
WORKDIR /src
COPY go.mod ./
COPY cmd ./cmd
RUN CGO_ENABLED=0 GOOS=linux go build -trimpath -ldflags="-s -w" -o /bot ./cmd/bot

FROM scratch
COPY --from=build /bot /bot
USER 65534
ENTRYPOINT ["/bot"]
