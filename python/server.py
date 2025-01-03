#!/usr/bin/env python3
# coding=utf-8

import argparse
import http.server
import pathlib
import socket
import ssl

from RangeHTTPServer import RangeRequestHandler


class RequestHandler(RangeRequestHandler):
    def finish(self) -> None:
        try:
            if not self.wfile.closed:
                self.wfile.flush()
                self.wfile.close()
        except socket.error:
            pass
        self.rfile.close()

    def handle(self) -> None:
        try:
            super().handle()
        except socket.error:
            pass


def parse_args():
    parser = argparse.ArgumentParser(
        description="Start a HTTP Server, support range requests.",
    )
    parser.add_argument(
        "cert_dir",
        type=str,
        help="Directory containing ca_cert.pem and ca_key.pem",
    )
    parser.add_argument(
        "--range",
        action="store_true",
        help="Enable range requests",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8070,
        help="Port number to listen to",
    )
    return parser.parse_args()


def server():
    args = parse_args()
    cert_dir = pathlib.Path(args.cert_dir)
    cert_file = cert_dir / "ca_cert.pem"
    key_file = cert_dir / "ca_key.pem"

    if args.range:
        handler = RequestHandler
    else:
        handler = http.server.SimpleHTTPRequestHandler
    httpd = http.server.HTTPServer(("0.0.0.0", args.port), handler)

    sslctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    sslctx.check_hostname = False
    sslctx.load_cert_chain(certfile=cert_file, keyfile=key_file)

    httpd.socket = sslctx.wrap_socket(
        httpd.socket,
        server_side=True,
    )
    print(f"Serving at port {args.port}")
    httpd.serve_forever()


if __name__ == "__main__":
    server()
