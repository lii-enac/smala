(use-package ido)

(add-to-list 'auto-mode-alist '("\\.sma\\'" . smala-mode))

(defvar smala-mode-hook nil)

(defvar smala-mode-map
  (let ((map (make-sparse-keymap)))
    (define-key map (kbd "C-c p") 'smala-property)
    (define-key map (kbd "C-c c") 'smala-component)
    (define-key map (kbd "C-c g") 'smala-gui-component)
    map)
  "Keymap for Smala major mode")


(defvar smala-font-lock
  (let* (
         (smala-keywords '("if" "then" "else""for" "while" "insert"
                           "int" "double" "string"
                           "remove" "move"
                           ))
         (smala-pp '("use" "import" "aka"
                     "_main_" "_define_"
                     "_action_" "_native_code_" "_native_java_"))
         (smala-types '("CCal" "NativeCall" "Native" "Alias" "DashArray"
                        "Process" "FSM" "State"))
         (smala-constants '("TRUE" "FALSE" "null"))
         (smala-builtins '("run" "stop" "delete" "notify" "merge" "dump"
                           "with" "from" "print"
                           "loadFromXML" "addChild" "addChildrenTo"
                           "find" "clone" "repeat"
                           "setInt" "setDouble" "setBool" "setString" "setRef"
                        ))

         (smala-keywords-regexp (regexp-opt smala-keywords 'words))
         (smala-pp-regexp (regexp-opt smala-pp 'words))
         (smala-types-regexp (regexp-opt smala-types 'words))
         (smala-constants-regexp (regexp-opt smala-constants 'words))
         (smala-builtins-regexp (regexp-opt smala-builtins 'words))
         )
    `((,smala-constants-regexp . font-lock-constant-face)
      ;; ("^[[:space:]]*[[:upper:]][[:alnum:]]*[[:space:]]+\\([[:lower]][[:alnum:]]*\\)[[:space:]]?{" . (1 font-lock-function-name-face))
      (,smala-types-regexp . font-lock-type-face)
      ("[[:upper:]][[:alnum:]]*" . font-lock-type-face)
      (,smala-builtins-regexp . font-lock-builtin-face)
      (,smala-keywords-regexp . font-lock-keyword-face)
      ("=>\\|=:>\\|::>\\|=:\\|::\\|<<\\|->\\|!->\\|->!\\|!->!"
       . font-lock-keyword-face)
      (,smala-pp-regexp . font-lock-preprocessor-face)
      ("(\\|)\\|{\\|}" . font-lock-doc-face)
      ;; note: order above matters, because once colored, that part won't change.
      ;; in general, put longer words first
      ))
  "Default highlighting expressions for SMALA mode")

(defvar smala-mode-syntax-table
  (let ((st (make-syntax-table)))
    (modify-syntax-entry ?_ "w" st)
    (modify-syntax-entry ?/ ". 124b" st)
    (modify-syntax-entry ?* ". 23" st)
    (modify-syntax-entry ?\n "> b" st)
    st)
  "Syntax table for smala-mode")

(defvar smala-tab-width 2)

(defun smala-indent-line ()
  "Indent current line as Smala code"
  (interactive)
  (beginning-of-line)
  (if (bobp)
      (indent-line-to 0)
    (let ((not-indented t) cur-indent)
      (if (looking-at "^[ \t]*}")
          (progn
            (save-excursion
              (forward-line -1)
              (setq cur-indent (- (current-indentation) smala-tab-width)))
            (if (< cur-indent 0)
                (setq cur-indent 0)))
        (save-excursion
          (while not-indented
            (forward-line -1)
            (if (looking-at "^[ \t]*}")
                (progn
                  (setq cur-indent (current-indentation))
                  (setq not-indented nil))
              (if (looking-at "^.*{")
                  (progn
                    (setq cur-indent (+ (current-indentation) smala-tab-width))
                    (setq not-indented nil))
                (if (bobp)
                    (setq not-indented nil)))))))
      (if cur-indent
          (indent-line-to cur-indent)
        (indent-line-to 0))
      )
    )
  )

(defvar smala-prettify-symbols-alist
  `(("->" . ?→) ("!->" . ?\u21A6) ("->!" . ?\u21E5) ;;("!->!" . )
    ("=>" . ?\u21D2) ;;("=:>" . )
    ("=:" . ?\u2255) ("::" . ?\u2237) ;;("::>" . )
    ("&&" . ?∧) ("||" . ?∨)
    ("<=" . ?≤) (">=" . ?≥) ("<>" . ?≠) ("==" . ?≡) ("!=" . ?≢)
    ))

(define-skeleton smala-property
  "Insert a Smala property"
  nil
  '(defvar-local prop nil)
  '(defvar-local str nil)
  '(interactive)
  '(let ((choices '("Bool" "Int" "Double" "String")))
     (setq-local prop
                 (message "%s" (ido-completing-read "GUI component:" choices ))))
  '(setq name (skeleton-read "Name: "))
  '(setq value (skeleton-read "Value: "))
  '(setq-local str (if (equal "String" prop) "\""))
  > prop " " name " (" str value str ")" \n _)

(define-skeleton smala-component
  "Define a new Smala component"
  nil
  '(setq name (skeleton-read "Name: "))
  > "Component " name " {" \n > _ "}" >)

(define-skeleton smala-gui-circle
  "Insert a Smala circle"
  nil
  '(setq name (skeleton-read "Name: "))
  '(setq cx (skeleton-read "Center-X: "))
  '(setq cy (skeleton-read "Center-Y: "))
  '(setq r (skeleton-read "Radius: "))
  > "Circle " name " (" cx ", " cy ", " r ")" \n _)

(define-skeleton smala-gui-fillcolor
  "Insert a Smala FillColor"
  nil
  '(setq name (skeleton-read "Name: "))
  '(setq r (skeleton-read "Red: "))
  '(setq g (skeleton-read "Green: "))
  '(setq b (skeleton-read "Blue: "))
  > "FillColor " name " (" r ", " g ", " b ")" \n _)

(define-skeleton smala-gui-rectangle
  "Insert a Smala rectangle"
  nil
  '(setq name (skeleton-read "Name: "))
  '(setq x (skeleton-read "X: "))
  '(setq y (skeleton-read "Y: "))
  '(setq w (skeleton-read "Width: "))
  '(setq h (skeleton-read "Height: "))
  '(setq rx (skeleton-read "X-Radius: "))
  '(setq ry (skeleton-read "Y-Radius: "))
  > "Rectangle " name " (" x ", " y ", " w ", " h ", " rx ", " ry ")" \n _)

(define-skeleton smala-gui-line
  "Insert a Smala line"
  nil
  '(setq name (skeleton-read "Name: "))
  '(setq x1 (skeleton-read "X1: "))
  '(setq y1 (skeleton-read "Y1: "))
  '(setq x2 (skeleton-read "X2: "))
  '(setq y2 (skeleton-read "Y2: "))
  > "Line " name " (" x1 ", " y1 ", " x2 ", " y2 ")" \n _)

(defun smala-gui-component ()
  "Prompt user to pick a component from Smala GUI library."
  (defvar-local comp nil)
  (interactive)
  (let ((choices '("Circle" "FillColor" "Rectangle" "Line")))
    (setq-local comp
                (message "%s" (ido-completing-read "GUI component:" choices )))
    (cond ((equal comp "Circle") (call-interactively 'smala-gui-circle))
          ((equal comp "FillColor") (call-interactively 'smala-gui-fillcolor))
          ((equal comp "Rectangle") (call-interactively 'smala-gui-rectangle))
          ((equal comp "Line") (call-interactively 'smala-gui-line))
          )
    ))

(define-derived-mode smala-mode prog-mode "Smala"
  "Major mode for editing Smala files."
  :syntax-table smala-mode-syntax-table
  (set (make-local-variable 'font-lock-defaults) '(smala-font-lock))
  (set (make-local-variable 'indent-line-function) 'smala-indent-line)
  (setq-local comment-start "/*")
  (setq-local comment-start-skip "/\\*+[ \t]*")
  (setq-local comment-end "*/")
  (setq-local comment-end-skip "[ \t]*\\*+/")
  (setq-local prettify-symbols-alist smala-prettify-symbols-alist)
  (prettify-symbols-mode t)
  )

(provide 'smala-mode)

