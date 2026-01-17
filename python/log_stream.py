#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import requests
import time
import argparse
import sys
import socket  # Needed for potential hostname resolution errors


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ESP32 HTTP Log Client")
    parser.add_argument(
        "--host",
        required=True,
        help="ESP32 web server hostname (e.g., myesp.local) or IP address",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.1,
        help="Polling interval in seconds (e.g., 0.1 for 100ms)",
    )
    parser.add_argument(
        "--timeout", type=float, default=2.0, help="Request timeout in seconds"
    )
    args = parser.parse_args()

    # Construct the URL for the log endpoint (assuming default HTTP port 80)
    log_url = f"http://{args.host}/logz"
    print(
        f"Polling logs from {log_url} every {args.interval:.2f} seconds...",
        file=sys.stderr,
    )

    session = requests.Session()  # Use a session for potential connection reuse

    try:
        while True:
            try:
                response = session.get(log_url, timeout=args.timeout)

                # Check if the request was successful (status code 200)
                # The ESP32 handler now always returns 200 OK.
                if response.status_code == 200:
                    # Get the response text and strip leading/trailing whitespace
                    log_message = response.text.strip()
                    # Only print if the message is not empty after stripping.
                    # An empty response means no log was available during this poll.
                    if log_message:
                        print(log_message)
                # Removed the check for 404, as the handler no longer sends it.
                else:
                    # Handle other unexpected HTTP errors
                    print(
                        f"Error: Received HTTP status {response.status_code} from {args.host}",
                        file=sys.stderr,
                    )
                    # Optional: Add a longer delay after non-200 errors
                    time.sleep(1)

            except requests.exceptions.Timeout:
                print(
                    f"Error: Request to {args.host} timed out after {args.timeout} seconds.",
                    file=sys.stderr,
                )
                time.sleep(1)  # Wait a bit longer after a timeout
            except requests.exceptions.ConnectionError as e:
                # This can happen if hostname resolution fails or server is down
                print(
                    f"Error: Could not connect to {args.host}. Details: {e}",
                    file=sys.stderr,
                )
                time.sleep(2)  # Wait longer after connection errors
            except requests.exceptions.RequestException as e:
                # Catch other potential requests library errors
                print(
                    f"Error: An unexpected request error occurred for {args.host}: {e}",
                    file=sys.stderr,
                )
                time.sleep(1)
            except socket.gaierror as e:
                # Specifically catch hostname resolution errors
                print(
                    f"Error: Could not resolve hostname {args.host}. Details: {e}",
                    file=sys.stderr,
                )
                time.sleep(5)  # Wait longer if hostname resolution fails

            # Wait for the specified interval before the next poll
            time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nClient stopped.", file=sys.stderr)
    finally:
        session.close()  # Close the session


if __name__ == "__main__":
    main()
