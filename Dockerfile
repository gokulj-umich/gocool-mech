FROM golang:1.22 AS build
WORKDIR /src
COPY go.mod ./
COPY cmd ./cmd
RUN CGO_ENABLED=0 go build -trimpath -ldflags="-s -w" -o /bot ./cmd/bot
FROM gcr.io/distroless/static
COPY --from=build /bot /bot
USER 65534
ENTRYPOINT ["/bot"]
