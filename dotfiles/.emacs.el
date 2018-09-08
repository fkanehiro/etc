;;Tex

(require 'package)

; Add package-archives
(add-to-list 'package-archives '("melpa" . "http://melpa.milkbox.net/packages/") t)
(add-to-list 'package-archives '("marmalade" . "http://marmalade-repo.org/packages/")) ; ついでにmarmaladeも追加

;; Added by Package.el.  This must come before configurations of
;; installed packages.  Don't delete this line.  If you don't want it,
;; just comment it out by adding a semicolon to the start of the line.
;; You may delete these explanatory comments.
(package-initialize)

(setq auto-mode-alist
      (cons (cons "\\.tex$" 'yatex-mode) auto-mode-alist))
(autoload 'yatex-mode "yatex""Yet Another LaTeX mode" t)
(setq tex-command "~/Library/TeXShop/bin/platex2pdf-utf8" dvi2-command "open -a Skim" bibtex-command "/Library/TeX/texbin/pbibtex")

(if (eq window-system 'mac) (require 'carbon-font))
					;(fixed-width-set-fontset "hirakaku_w3" 18)
(when (and (>= emacs-major-version 24) (not (null window-system)))
  (let* ((font-family "Menlo")
         (font-size 20)
         (font-height (* font-size 10))
         (jp-font-family "ヒラギノ角ゴ ProN"))
    (set-face-attribute 'default nil :family font-family :height font-height)
    (let ((name (frame-parameter nil 'font))
          (jp-font-spec (font-spec :family jp-font-family))
          (jp-characters '(katakana-jisx0201
                           cp932-2-byte
                           japanese-jisx0212
                           japanese-jisx0213-2
                           japanese-jisx0213.2004-1))
          (font-spec (font-spec :family font-family))
          (characters '((?\u00A0 . ?\u00FF)    ; Latin-1
                        (?\u0100 . ?\u017F)    ; Latin Extended-A
                        (?\u0180 . ?\u024F)    ; Latin Extended-B
                        (?\u0250 . ?\u02AF)    ; IPA Extensions
                        (?\u0370 . ?\u03FF)))) ; Greek and Coptic
      (dolist (jp-character jp-characters)
        (set-fontset-font name jp-character jp-font-spec))
      (dolist (character characters)
        (set-fontset-font name character font-spec))
      (add-to-list 'face-font-rescale-alist (cons jp-font-family 1.2)))))


;; ウィンドウ設定

(if window-system (progn
		    (setq initial-frame-alist '((width . 80) (height . 54) (left . 60) (top . 0)))
;		    (set-background-color "Black")
;		    (set-foreground-color "White")
;		    (set-cursor-color "Gray")
		    ))


;; メニューバーを隠す

;(tool-bar-mode -1)

(defun c-mode-configure ()
  (c-set-style "cc-mode")
  (setq c-basic-offset 4)
  (setq indent-tabs-mode nil)
  )

(add-hook 'c-mode-hook 'c-mode-configure)
(add-hook 'c++-mode-hook 'c-mode-configure)
(define-key global-map [?¥] [?\\])  ;; ¥の代わりにバックスラッシュを入力する

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

;(global-git-gutter-mode t)

;; ivy設定
(require 'ivy)
(ivy-mode 1)
(setq ivy-use-virtual-buffers t)
(setq enable-recursive-minibuffers t)
(setq ivy-height 30) ;; minibufferのサイズを拡大！（重要）
(setq ivy-extra-directories nil)
(setq ivy-re-builders-alist
      '((t . ivy--regex-plus)))

;; counsel設定
(require 'counsel)
(global-set-key (kbd "M-x") 'counsel-M-x)
;(global-set-key (kbd "C-x C-f") 'counsel-find-file) ;; find-fileもcounsel任せ！
(defvar counsel-find-file-ignore-regexp (regexp-opt '("./" "../")))

(global-set-key "\C-s" 'swiper)
(defvar swiper-include-line-number-in-search t) ;; line-numberでも検索可能

(show-paren-mode t)

;; Highlight the line we are currently on
;(global-hl-line-mode t)

(global-set-key (kbd "C-x g") 'magit-status)

(custom-set-variables
 ;; custom-set-variables was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 '(package-selected-packages
   (quote
    (magit erc-image rtags powerline git-gutter counsel company-jedi company-irony))))
(custom-set-faces
 ;; custom-set-faces was added by Custom.
 ;; If you edit it by hand, you could mess it up, so be careful.
 ;; Your init file should contain only one such instance.
 ;; If there is more than one, they won't work right.
 )

(require 'cmake-mode); Add cmake listfile names to the mode list.
(setq auto-mode-alist
	  (append
	   '(("CMakeLists\\.txt\\'" . cmake-mode))
	   '(("\\.cmake\\'" . cmake-mode))
	   auto-mode-alist))
