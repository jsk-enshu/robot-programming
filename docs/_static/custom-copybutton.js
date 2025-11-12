// カスタムプロンプト除外処理
document.addEventListener('DOMContentLoaded', function() {
    // sphinx-copybuttonが初期化された後に実行
    setTimeout(function() {
        // 全てのコードブロックに対して処理
        document.querySelectorAll('div.highlight pre').forEach(function(pre) {
            // Lispブロック内の irteusgl$ を含む span.nv 要素を探す
            const nvSpans = pre.querySelectorAll('span.nv');

            nvSpans.forEach(function(span) {
                if (span.textContent === 'irteusgl$') {
                    // irteusgl$ を選択不可にする
                    span.style.userSelect = 'none';
                    span.style.webkitUserSelect = 'none';
                    span.style.mozUserSelect = 'none';
                    span.style.msUserSelect = 'none';

                    // 後ろの空白も選択不可にする
                    const nextNode = span.nextSibling;
                    if (nextNode && nextNode.nodeType === Node.ELEMENT_NODE &&
                        nextNode.classList.contains('w') && nextNode.textContent === ' ') {
                        nextNode.style.userSelect = 'none';
                        nextNode.style.webkitUserSelect = 'none';
                        nextNode.style.mozUserSelect = 'none';
                        nextNode.style.msUserSelect = 'none';
                    }
                }
            });
        });
    }, 100);

    // 全ての画像にライトボックス機能を追加
    setTimeout(function() {
        // figure要素内の画像を全て取得
        document.querySelectorAll('figure img').forEach(function(img, index) {
            // 画像を<a>タグでラップ
            if (!img.parentElement.tagName || img.parentElement.tagName.toLowerCase() !== 'a') {
                const link = document.createElement('a');
                link.href = img.src;
                link.setAttribute('data-lightbox', 'image-gallery');

                // 画像のキャプションを取得してタイトルとして設定
                const figure = img.closest('figure');
                if (figure) {
                    const caption = figure.querySelector('figcaption .caption-text');
                    if (caption) {
                        link.setAttribute('data-title', caption.textContent);
                    }
                }

                // 画像をリンクでラップ
                img.parentNode.insertBefore(link, img);
                link.appendChild(img);

                // 画像をクリック可能にするためのスタイル
                img.style.cursor = 'pointer';
            }
        });
    }, 200);
});
