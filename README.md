# 99484A 2024-25 season code

[![Made with vexide badge](https://img.shields.io/badge/Made%20with-vexide-e6c85c.svg?style=for-the-badge&logo=data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACAAAAAgCAMAAABEpIrGAAAAIGNIUk0AAHomAACAhAAA+gAAAIDoAAB1MAAA6mAAADqYAAAXcJy6UTwAAAHmUExURQAAAOXIW+bIXObIXNfb6dfb6djb6djb6eXIXOXIXObIXObIXObIXOXIXNjb6djb6djb6djb6eXIW+XIW+bIXObIW+bIW+bIW+bHW9jb6djb6djb6Njb6OXHXObIXObIXObHXObIW+XIW+XIW+bHXObHW9jb6Njb6dja6Nfb6uXHXOXHXObIXObIXObIXOXIW+bIW+bHXObHW+bIW9fb79jb6Njb6djb6djb6Njb6eXIXOXHXObIXObIXObHXObHW+bHXObIW+bIW9fb6djb6dja6Nfa6eXHXOXIW+bIW+bIW+bHW+bHXObHXObHXObIXObIXObIW+bHW+bIW+bHW+bHXObIW+bIXObIXOXHXOXHW+bHXOXIXOXHXOXHXObHXObHXObIW+XHW+bHXObIW+XHXObHXOXIXObIXObHW+bHW+XIXOXIXObIXObIW+bIW+bIXObIW+bIXObIW+bIW+bIXObIW+bIXOXIXObIXOXIW+bIW+bIW+bIW+bIXObHXOXHXOXHXObHXObIXObIXOXIW+XIW+XIXOXHXOXHXObHXObHXObHXObIXObIXObIXObIXObHXObIW+bIW+XHXObHXOXHXObIXObIXObIXObIXObIXObIW+bIXObHXObIXNjb6QAAAKYhk3UAAACfdFJOUwDh5EMBmudIQ+Fs7NM3PtzYMzbTAmLv0TRG4u1GNNHvAwNn8c4xUO5hAjDO8WcDBGvyyy4BWe3xZQMty/IEBHD0yCsDVloEKsj0cAQFdPXEKCjEdAUGePbBJSWYB333vSEhvcUlCIH5yL0jCYX6uiAKivjVth4Mjvv7jhmJ+rIbDZL8/JINC437rRwOl/2XDv66EJqaEBBui4p7Fa6uFUfmhQMAAAABYktHRACIBR1IAAAACXBIWXMAAA7DAAAOwwHHb6hkAAAAB3RJTUUH6QEaBTUsaXkSHwAAAURJREFUOMtjYBgZgHE+CDAxMzCwsC4AATZ2sDgHE1iCk4GLG8zg4WVg4OMHqxAQBMoLCYOFubkYRETFwExxCQYGSSmQAmkZBgZZObCgvKgCA4OikjKYo6LKwKDGtmCBuoYmg5Y2WEhHVw9km76BIZhrZMxgYmpmbmHJYGUNFrDhsoW4087eASzg6MTg7OLqxuDuAeZ6ennDfOLj6wcW8g8A8QKDwBy/4BCEX0PDwsGCEZEMDFERYOb8aB/k0IiJjQOLxickJkHk5yenoIRXalo6WDgjYz4MZGahqMjOyZ2PBvLyUVQUFMJVFBUXgemSUhQVZeUVEPnKquqaWjCrrh5FRUNjE0i0uaWVoa29A6yiswtFRXdP7/z5vX39YMXNIAUTJqLG/aTJvb1TpoKZ06bPmDlz5oxZaKlj9py58wY0eQ5aAACvpKcarQF0GAAAABl0RVh0U29mdHdhcmUAd3d3Lmlua3NjYXBlLm9yZ5vuPBoAAAAASUVORK5CYII=)](https://vexide.dev)
[![Contact us button](https://img.shields.io/badge/Contact%20us-c33e2e.svg?style=for-the-badge)](https://github.com/orgs/doxa-robotics/discussions/new?category=general)

> ![NOTE]
> 
> This repository is archived. Please create a discussion with the button above if you have any questions.
> このリポジトリーはアーカイブになりました。質問があれば、上のボタンを押してください。

## English

This repository contains the source of 99484A's competition code for the 2024-25 school year. It is written in Rust using the vexide framework, which we also actively contribute to. 

We ranked first in Japan for skills and tournaments this year with this codebase. Worldwide, we are a mid-level team with experience with basic control theory. Most of the business logic is extracted into a separate library which is linked from our org page. We ranked around the 65th percentile at VEX Worlds.

## 日本語

このリポジトリはDOXA Roboticsの2024-2025学年のコードです。Rustのプログラミング言語で書いて、PIDやPure pursuitなどの制御理論を使っています。

## Copyright

&copy;2024-25 zabackary, rh0820, and Spacyhula. All rights reserved. Please see the [LICENSE](./LICENSE) file for more details.

## Driver control documentation

- Split arcade drive
- R1 to intake
- L1 to outtake
- Y to partial intake
- X to toggle lady brown down/intake/max expansion
  - When going from intake to max expansion, intake is turned on for 500ms and
    expansion is delayed for 200ms.
- L2/R2 to manually control lady brown
- A toggles clamp
- B toggles doinker
