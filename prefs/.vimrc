"--------------------
"Vundle
"--------------------

set nocompatible
filetype off


set rtp+=~/.vim/bundle/Vundle.vim
"set runtimepath^=~/.vim/bundle/ctrlp.vim
call vundle#begin()

"Plugins
Plugin 'gmarik/Vundle.vim'
Plugin 'christoomey/vim-tmux-navigator'
Plugin 'git://github.com/jiangmiao/auto-pairs'
Plugin 'git://github.com/vim-scripts/DfrankUtil'
Plugin 'git://github.com/vim-scripts/vimprj'
Plugin 'git://github.com/vim-scripts/indexer.tar.gz.git'

"Syntax & Color
Plugin 'justinmk/vim-syntax-extra'
Plugin 'octol/vim-cpp-enhanced-highlight'
Plugin 'python-syntax'
Plugin 'rust-lang/rust.vim'
Plugin 'digitaltoad/vim-jade'

call vundle#end()
"filetype plugin indent on
filetype on
filetype plugin on
filetype indent on


"--------------------
"Configuration
"--------------------
set hidden
set backspace=indent,eol,start
set background=dark
set autoindent
set smartindent
"set expandtab
set tabstop=4
set shiftwidth=4
set number
set relativenumber
set numberwidth=1
"set paste
set t_ut=
set t_Co=256
"colorscheme oceannight
"colorscheme obsidian2
colorscheme tayra
"colorscheme OceanicNext
"colorscheme Kafka
"colorscheme termschool
"colorscheme ego
"colorscheme onedark
"colorscheme materialbox
"colorscheme solarized
"colorscheme antares
syntax on

let g:netrw_liststyle=3

"Language specific
let g:cpp_class_scope_highlight=1 
let g_cpp_experimental_template_highlight=1


map <C-\> :tab split<CR>:exec("tag ".expand("<cword>"))<CR>
map <A-]> :vsp <CR>:exec("tag ".expand("<cword>"))<CR>

map <Enter> o<ESC>
map <S-ENTER> O<ESC>


au! FileType python setl nosmartindent
au BufRead,BufNewFile *.cl,*cu,*cuh set filetype=c
