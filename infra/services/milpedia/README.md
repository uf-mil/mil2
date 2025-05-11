# MILpedia (wiki) containerization

This container is responsible for running the MILpedia service of the Machine Intelligence
Laboratory. Under the hood, it uses [Mediawiki](https://www.mediawiki.org/wiki/MediaWiki) as the wiki engine and [MariaDB](https://mariadb.org/) as the database engine.

An example compose file is provided in `docker-compose.yaml`.

## Environment variables

No environment variables have default values, and all must be supplied in order to
properly run the wiki. The variables that can be supplied include:

| Name | Example | Meaning |
| - | - | - |
| `SMTP_USER` | `email-user` | The username used to sign into the SMTP server. |
| `SMTP_PASS` | `email-pass` | The password used to sign into the SMTP server. |
| `SMTP_HOST` | `smtp.example.com` | The hostname of the SMTP server. |
| `SMTP_EMAIL` | `user@example.com` | The email address used to send emails from. |
| `MARIADB_USER` | `wiki` | The username used to sign into the MariaDB server. |
| `MARIADB_PASS` | `b4K$cc0@_2E` | The password used to sign into the MariaDB server. |
| `MARIADB_ROOT_PASS` | `b4K$cc0@_2E` | The password used to sign into the MariaDB server as root. |
| `WG_SECRET_KEY` | `bca1b2c3d4e5f6g7h8i9j0` | The secret key used to sign cookies. |
| `WG_UPGRADE_KEY` | `bca1b2c3d4e5f6g7h8i9j0` | The key used to upgrade the wiki. |
| `DISCORD_WEBHOOK_URL` | `https://discord.com/api/webhooks/1234567890/abcdefghijklmnopqrstuvwxyz` | The URL of the Discord webhook used to send notifications. |
