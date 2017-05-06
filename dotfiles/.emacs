(require 'package)

; Add package-archives
(add-to-list 'package-archives '("melpa" . "http://melpa.milkbox.net/packages/") t)
(add-to-list 'package-archives '("marmalade" . "http://marmalade-repo.org/packages/")) ; ついでにmarmaladeも追加

; Initialize
(package-initialize)

(require 'company)
(global-company-mode) ; 全バッファで有効にする 
(setq company-idle-delay 0) ; デフォルトは0.5
(setq company-minimum-prefix-length 2) ; デフォルトは4
(setq company-selection-wrap-around t) ; 候補の一番下でさらに下に行こうとすると一番上に戻る

(require 'irony)
(add-hook 'c-mode-hook 'irony-mode)
(add-hook 'c++-mode-hook 'irony-mode)
(add-hook 'objc-mode-hook 'irony-mode)
(add-hook 'irony-mode-hook 'irony-cdb-autosetup-compile-options)
(add-to-list 'company-backends 'company-irony) ; backend追加

(require 'jedi-core)
(setq jedi:complete-on-dot t)
(setq jedi:use-shortcuts t)
(add-hook 'python-mode-hook 'jedi:setup)
(add-to-list 'company-backends 'company-jedi) ; backendに追加

(when (require 'rtags nil 'noerror)
  (add-hook 'c-mode-common-hook
            (lambda ()
	      (setq rtags-path "/usr/local/bin")
	      (local-set-key (kbd "M-.") 'rtags-find-symbol-at-point)
	      (local-set-key (kbd "M-;") 'rtags-find-symbol)
	      (local-set-key (kbd "M-@") 'rtags-find-references)
	      (local-set-key (kbd "M-,") 'rtags-location-stack-back))))

(require 'powerline)
(powerline-default-theme)

(require 'linum)            ;\左に行番号表示
(global-linum-mode)
