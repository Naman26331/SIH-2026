# 20 — Cybersecurity & Trusted Decentralized Communication

## Purpose
Protect robot-to-robot coordination from forged, stale, duplicated, or unauthorized messages.

## Threat Model
Consider fake identities, forged intents, replayed messages, malformed payloads, message flooding, unauthorized commands, and compromised robots.

## Message Metadata
```text
message_id
sender_id
timestamp
sequence_number
message_type
payload
authentication_tag/signature
```

## Authentication
Use authenticated robot identities such as mutual TLS, signed messages, device certificates, or rotating credentials.

## Replay Protection
Reject old timestamps, previously processed message IDs, and invalid sequence numbers.

## Authorization
Separate permissions for telemetry, task bidding, reservations, task assignment, and emergency commands.

## Validation
Validate coordinates, path length, timestamps, IDs, priorities, battery values, and message size.

## Rate Limiting
Limit abnormal message frequency to reduce flooding risk.

## Safety Boundary
Security must never delay emergency stopping. Safety-critical action remains local.

## SIH Demo
Simulate a fake robot message → authentication fails → message is rejected → event logged → fleet continues safely.
